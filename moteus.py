#!/usr/bin/env python3

import math
import threading
import time
from enum import Enum
from typing import Dict, List, TypedDict

import moteus
import rospy
from msg import ArmOpenLoopCmd, ArmPosition


class MoteusEvent(Enum):
    TURN_OFF = 0
    TURN_ON = 1


class ControlData:

    ff_torque: float
    kd_scale: float
    kp_scale: float
    max_torque: float
    position: float
    velocity: float

    def __init__(self) -> None:
        self.ff_torque = 0.0
        self.kd_scale = 1.0
        self.kp_scale = 0.0
        self.max_torque = 0.5
        self.position = math.nan
        self.velocity = 0.0


class ControllerDict(TypedDict, total=False):
    name: str
    node_id: int
    inversion: int
    limits: List[str]


class Controller:

    c: moteus.Controller
    control_data: ControlData
    inversion: int
    limit_negative: bool
    limit_positive: bool
    lock: threading.Lock
    node_id: int
    query_data: moteus.cmd

    def __init__(self, controller: ControllerDict) -> None:
        self.c = None
        self.control_data = ControlData()
        self.has_limit_negative = False
        self.has_limit_positive = False
        if "limits" in controller:
            for limit in controller["limits"]:
                if limit == "neg":
                    self.has_limit_negative = True
                elif limit == "pos":
                    self.has_limit_positive = True
        self.inversion = controller["inversion"]
        self.limit_negative = False
        self.limit_positive = False
        self.lock = threading.Lock()
        self.node_id = controller["node_id"]
        self.query_data = None

    def __enter__(self):
        self.turn_on()

    def __exit__(self, exc_type, exc_value, traceback):
        self.turn_off()


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

    def set_position_control(self, pos, vel=0.0, max_tor=0.5, ff_tor=0.0, kp=1.0, kd=1.0) -> None:
        self.lock.acquire()
        self.control_data.position = pos
        self.control_data.velocity = vel
        self.control_data.max_torque = max_tor
        self.control_data.ff_torque = ff_tor
        self.control_data.kp_scale = kp
        self.control_data.kd_scale = kd
        self.lock.release()

    def set_velocity_control(self, vel=0.0, max_tor=0.5, ff_tor=0.0, kd=1.0) -> None:
        self.lock.acquire()
        self.control_data.position = math.nan
        self.control_data.velocity = vel
        self.control_data.max_torque = max_tor
        self.control_data.ff_torque = ff_tor
        self.control_data.kp_scale = 0.0
        self.control_data.kd_scale = kd
        self.lock.release()

    def set_torque_control(self, tor=0.0) -> None:
        self.lock.acquire()
        self.control_data.position = math.nan
        self.control_data.velocity = 0
        self.control_data.max_torque = abs(tor)
        self.control_data.ff_torque = tor
        self.control_data.kp_scale = 0.0
        self.control_data.kd_scale = 0.0
        self.lock.release()

    def update_limit(self) -> None:
        if not self.has_limit_negative and not self.has_limit_positive:
            return
        bytes_object = self.c.read_gpio()
        # TODO - Figure out how to parse this bytes object
        # TODO - Figure out if it should be active low or high
        self.limit_negative = False  # TODO - eventually change
        self.limit_positive = False # TODO - eventually change


    def update_speed(self) -> None:
        self.lock.acquire()
        if (
            (self.limit_negative and self.control_data.velocity < 0)
            or (self.limit_positive and self.control_data.velocity > 0)
        ):
            self.set_velocity_control(vel=0.0)
            # TODO - Fix for closed. This code only protects open-loop.
            # This is because vel can be set as 0.0 in positive control mode.
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
        self.lock.release()

    def turn_off(self) -> None:
        # this also clears errors
        self.query_data = self.c.make_stop()
        self.set_velocity_control(0.0)

    def turn_on(self) -> None:
        # NOTE: online it says that it connects to an arbitrary can port, prioritizing
        # the fdcanbus. So I'd assume if we just connected to can, it would just connect to that.
        # We can explore this later.
        self.c = moteus.Controller(id=self.node_id)
        self.set_velocity_control(0.0)


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

    controller: Controller
    state: State

    def __init__(self, controller: ControllerDict) -> None:
        self.controller = Controller(controller)
        self.state = OffState()

    def is_on(self) -> bool:
        return str(self.state) == "OnState"

    def update_bridge(self) -> None:
        if str(self.state) == "OnState":
            errors = self.controller.has_errors()
            if errors:
                self.state.on_event(MoteusEvent.TURN_OFF, self.controller)
                return
            self.controller.update_limit()
            self.controller.update_speed()
        else:
            self.state.on_event(MoteusEvent.TURN_ON, self.controller)


class ControllerMap:

    id_by_controller_name: Dict[str, int]
    live_controller_name_by_id: Dict[int, str]
    moteus_bridge_by_controller_name: Dict[str, MoteusBridge]

    def __init__(self) -> None:
        self.live_controller_name_by_id = {}
        self.id_by_controller_name = {}
        self.moteus_bridge_by_controller_name = {}

        list_of_controllers = rospy.get_param("/motors/controllers")

        for controller in list_of_controllers:
            controller_name = controller["name"]
            controller_id = controller["node_id"]
            self.id_by_controller_name[controller_name] = controller_id
            self.moteus_bridge_by_controller_name[controller_name] = (
                MoteusBridge(controller)
            )

    def is_controller_live(self, name) -> bool:
        return name == self.live_controller_name_by_id[self.id_by_controller_name[name]]

    def make_live(self, name) -> None:
        moteus_bridge = self.moteus_bridge_by_controller_name[name]
        moteus_bridge.state.on_event(
            MoteusEvent.TURN_ON,
            moteus_bridge.controller
        )
        self.live_controller_name_by_id[self.id_by_controller_name[name]] = name


class ROSHandler:

    _arm_pos_pub: rospy.Publisher
    _controller_map: ControllerMap
    _start_time: float

    def __init__(self) -> None:
        self._controller_map = ControllerMap()
        self._start_time = time.process_time()
        self._arm_pos_pub = rospy.Publisher("arm_position", ArmPosition, queue_size=1)
        rospy.Subscriber("arm_open_loop_cmd", ArmOpenLoopCmd, self._arm_open_loop_cmd_callback)
        rospy.Subscriber("arm_closed_loop_cmd", ArmPosition, self._arm_closed_loop_cmd_callback)

    def update_all_controllers(self) -> None:
        for node_id in self._controller_map.live_controller_name_by_id:
            controller_name = self._controller_map.live_controller_name_by_id[node_id]
            lost_comms = time.process_time() - self._start_time > 1.0
            moteus_bridge = self._controller_map.moteus_bridge_by_controller_name[controller_name]
            if lost_comms:
                moteus_bridge.state.on_event(MoteusEvent.TURN_OFF, moteus_bridge.controller)
            moteus_bridge.update_bridge()

    def publish_all_controllers(self) -> None:
        self._publish_arm_pos_data()

    def _arm_closed_loop_cmd_callback(self, ros_msg: ArmPosition) -> None:
        arm_names = ["ARM_A", "ARM_B", "ARM_C", "ARM_D", "ARM_E", "ARM_F"]
        for index, name in enumerate(arm_names):
            self._set_controller_position_control(name, ros_msg.joints[index])

    def _arm_open_loop_cmd_callback(self, ros_msg: ArmOpenLoopCmd) -> None:
        arm_names = ["ARM_A", "ARM_B", "ARM_C", "ARM_D", "ARM_E", "ARM_F"]
        for index, name in enumerate(arm_names):
            self._set_controller_velocity_control(name, ros_msg.throttles[index])

    def _publish_arm_pos_data(self) -> None:
        ros_msg = ArmPosition()
        arm_names = ["ARM_A", "ARM_B", "ARM_C", "ARM_D", "ARM_E", "ARM_F"]
        for index, name in enumerate(arm_names):
            ros_msg.joints[index] = (
                self._controller_map.moteus_bridge_by_controller_name[name].controller.get_position()
            )
        self._arm_pos_pub.publish(ros_msg)

    def _set_controller_position_control(self, name, position) -> None:
        if not self._controller_map.is_controller_live(name):
            self._controller_map.make_live(name)
        moteus_bridge = self._controller_map.moteus_bridge_by_controller_name[name]
        moteus_bridge.controller.set_position_control(pos=position)

    def _set_controller_torque_control(self, name, torque) -> None:
        if not self._controller_map.is_controller_live(name):
            self._controller_map.make_live(name)
        moteus_bridge = self._controller_map.moteus_bridge_by_controller_name[name]
        moteus_bridge.controller.set_torque_control(tor=torque)

    def _set_controller_velocity_control(self, name, velocity) -> None:
        if not self._controller_map.is_controller_live(name):
            self._controller_map.make_live(name)
        moteus_bridge = self._controller_map.moteus_bridge_by_controller_name[name]
        moteus_bridge.controller.set_velocity_control(vel=velocity)


def main():
    rospy.init_node(f"moteus")
    with ROSHandler() as ros:
        threading._start_new_thread(ros.update_all_controllers, ())
        threading._start_new_thread(ros.publish_all_controllers, ())
        rospy.spin()
    exit()

if __name__ == "__main__":
    main()
