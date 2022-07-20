#!/usr/bin/env python3

import math
import threading
import time
from enum import Enum
from typing import Dict

import moteus
import rospy

from msg import ArmOpenLoopCmd, ArmPosition


class MoteusEvent(Enum):
    TURN_OFF = 0
    TURN_ON = 1


class ControlData:

    position: float
    velocity: float
    max_torque: float
    ff_torque: float
    kp_scale: float
    kd_scale: float

    def __init__(self) -> None:
        self.position = math.nan
        self.velocity = 0.0
        self.max_torque = 0.5
        self.ff_torque = 0.0
        self.kp_scale = 0.0
        self.kd_scale = 1.0
        self.set_velocity(0.0)

    def set_position(self, pos, vel=0.0, max_tor=0.5, ff_tor=0.0, kp=1.0, kd=1.0) -> None:
        self.position = pos
        self.velocity = vel
        self.max_torque = max_tor
        self.ff_torque = ff_tor
        self.kp_scale = kp
        self.kd_scale = kd

    def set_velocity(self, vel=0.0, max_tor=0.5, ff_tor=0.0, kd=1.0) -> None:
        self.position = math.nan
        self.velocity = vel
        self.max_torque = max_tor
        self.ff_torque = ff_tor
        self.kp_scale = 0.0
        self.kd_scale = kd

    def set_torque(self, tor=0.0) -> None:
        self.position = math.nan
        self.velocity = 0
        self.max_torque = abs(tor)
        self.ff_torque = tor
        self.kp_scale = 0.0
        self.kd_scale = 0.0


class Controller:

    can_id: int
    c: moteus.Controller
    inversion: int
    query_data: moteus.cmd
    control_data: ControlData

    def __init__(self, can_id, inversion) -> None:
        self.can_id = can_id
        self.c = None
        self.inversion = inversion
        self.query_data = None
        self.control_data = ControlData()

    def __enter__(self):
        self.turn_on()

    def __exit__(self, exc_type, exc_value, traceback):
        self.turn_off()

    def update(self) -> None:
        self.query_data = self.c.make_position(
            position=self.control_data.position,
            velocity=self.control_data.velocity * self.inversion,
            feedforward_torque=self.control_data.ff_torque,
            kp_scale=self.control_data.kp_scale,
            kd_scale=self.control_data.kd_scale,
            maximum_torque=self.control_data.max_torque,
            watchdog_timeout=10,
            query=True,
        )

    def turn_off(self) -> None:
        # this also clears errors
        self.c.make_stop()
        self.control_data.set_velocity(0.0)

    def turn_on(self) -> None:
        # NOTE: online it says that it connects to an arbitrary can port, prioritizing
        # the fdcanbus. So I'd assume if we just hooked up can, it would just connect to that.
        # We can explore this later.
        self.c = moteus.Controller(id=self.can_id)
        self.control_data.set_velocity(0.0)

    def get_position(self) -> float:
        if self.query_data is None:
            return 0

        return self.query_data.values[moteus.Register.POSITION]

    def get_velocity(self) -> float:
        if self.query_data is None:
            return 0

        return self.query_data.values[moteus.Register.VELOCITY]

    def has_errors(self) -> bool:
        if self.query_data is None:
            return False

        fault_value = self.query_data.values[moteus.Register.FAULT]
        return fault_value != 0


class State(object):
    def __init__(self) -> None:
        print("Processing current state:", str(self))

    def on_event(self, event: MoteusEvent, controller: Controller):
        """Handles events that are delegated to this State."""

    def __repr__(self) -> str:
        """Makes it so __str__ method can describe the State."""
        return self.__str__()

    def __str__(self) -> str:
        """Returns the name of the State."""
        return self.__class__.__name__


class OnState(State):

    def on_event(self, event: MoteusEvent, controller: Controller) -> State:
        if event == MoteusEvent.TURN_OFF:
            controller.turn_off()
            return OffState()
        return self


class OffState(State):
    def on_event(self, event: MoteusEvent, controller: Controller) -> State:
        if event == MoteusEvent.TURN_ON:
            controller.turn_on()
            return OnState()
        return self


class MoteusBridge:

    state: State
    name: str
    controller: Controller

    def __init__(self, name, id, inversion) -> None:
        self.state = OffState()
        self.name = name
        self.controller = Controller(id, inversion)

    def update(self) -> None:
        if str(self.state) == "OnState":
            errors = self.controller.has_errors()

            if errors:
                self.state.on_event(MoteusEvent.TURN_OFF, self.controller)
                return
        else:
            self.state.on_event(MoteusEvent.TURN_ON, self.controller)

    def is_on(self) -> bool:
        return str(self.state) == "OnState":


class ControllerMap:
    live_controller_name_by_id: Dict[int, str]
    id_by_controller_name: Dict[str, int]
    moteus_bridge_by_controller_name: Dict[str, MoteusBridge]

    def __init__(self) -> None:
        self.live_controller_name_by_id = {}
        self.id_by_controller_name = {}
        self.moteus_bridge_by_controller_name = {}

        list_of_controllers = rospy.get_param("/motors/controllers")

        for controller in list_of_controllers:
            controller_name = controller["name"]
            controller_id = controller["id"]
            controller_inversion = controller["controller_inversion"]
            self.id_by_controller_name[controller_name] = controller_id
            self.moteus_bridge_by_controller_name[controller_id] = (
                MoteusBridge(controller_name, controller_id, controller_inversion)
            )

    def is_controller_live(self, name) -> bool:
        return name == self.live_controller_name_by_id[self.id_by_controller_name[name]]

    def make_live(self, name) -> None:
        self.moteus_bridge_by_controller_name[name].state.on_event(MoteusEvent.TURN_ON)
        self.live_controller_name_by_id[self.id_by_controller_name[name]] = name


class ROSHandler:

    controller_map: ControllerMap
    start_time: float
    arm_pos_pub: rospy.Publisher

    def __init__(self) -> None:
        self.controller_map = ControllerMap()
        self.start_time = time.process_time()
        self.arm_pos_pub = rospy.Publisher("arm_position", ArmPosition, queue_size=1)
        rospy.Subscriber("arm_open_loop_cmd", ArmOpenLoopCmd, self.arm_open_loop_cmd_callback)

    def update_all_controllers(self) -> None:
        for can_id in self.controller_map.live_controller_name_by_id:
            controller_name = self.controller_map.live_controller_name_by_id[can_id]

            lost_comms = time.process_time() - self.start_time > 1.0
            if lost_comms:
                self.controller_map.moteus_bridge_by_controller_name[controller_name].state.on_event(MoteusEvent.TURN_OFF)
            self.controller_map.moteus_bridge_by_controller_name[controller_name].update()

    def publish_all_controllers(self) -> None:
        self.publish_arm_pos_data()

    def publish_arm_pos_data(self) -> None:
        ros_msg = ArmPosition()
        arm_names = ["ARM_A", "ARM_B", "ARM_C", "ARM_D", "ARM_E", "ARM_F"]
        for index, name in enumerate(arm_names):
            ros_msg.joints[index] = self.controller_map.moteus_bridge_by_controller_name[name].controller.get_position()
        self.arm_pos_pub.publish(ros_msg)

    def set_controller_torque(self, name, torque) -> None:
        if not self.controller_map.is_controller_live(name):
            self.controller_map.make_live(name)
        self.controller_map.moteus_bridge_by_controller_name[name].controller.control_data.set_torque(tor=torque)

    def set_controller_velocity(self, name, velocity) -> None:
        if not self.controller_map.is_controller_live(name):
            self.controller_map.make_live(name)
        self.controller_map.moteus_bridge_by_controller_name[name].controller.control_data.set_velocity(vel=velocity)

    def set_controller_position(self, name, position) -> None:
        if not self.controller_map.is_controller_live(name):
            self.controller_map.make_live(name)
        self.controller_map.moteus_bridge_by_controller_name[name].controller.control_data.set_position(pos=position)

    def arm_open_loop_cmd_callback(self, ros_msg: ArmOpenLoopCmd) -> None:
        arm_names = ["ARM_A", "ARM_B", "ARM_C", "ARM_D", "ARM_E", "ARM_F"]
        for index, name in enumerate(arm_names):
            self.controller_map.make_live(name)
            self.controller_map.moteus_bridge_by_controller_name[name].controller.set_velocity(ros_msg.throttle[index])


def main():
    ros_handler = ROSHandler()
    threading._start_new_thread(bridge.update_all_controllers, ())
    threading._start_new_thread(bridge.publish_all_controllers, ())

if __name__ == "__main__":
    main()
