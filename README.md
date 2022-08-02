# Brushless Test


## Overview

This repository contains code that could potentially be used for a system that controls various BLDC motors using the moteus controller with ROS.

---

## System

A Jetson Xavier NX will run the code. It will listen to ROS messages for either open-loop or closed-loop commands.

The Jetson Xavier will be connected to 6 or more moteus controllers via CAN-FD wiring. 

---

## Setup

One must make sure that all the motor controllers are assigned their proper unique ID. Instructions can be found online.

---

## Testing Plan for Components

There should be a testing process when testing individual components. Here is a general idea.

1. Control the motor using tview and the fdcanusb in velocity and position control mode.
2. Control the motor using code and calling the proper ID.
3. Control two motors daisy chained together.

---

## Setting up the fdcanusb

If the Jetson and/or the CAN transceiver is not available/ready, the fdcanusb can be used directly with one of the laptops. The wiring for the fdcanusb is straight forward. One end of the fdcanusb is USB micro which connects straight to the Jetson/laptop. The other end of the fdcanusb is a DB9 connector, which can easily be connected by searching up "can db9" on Google. Summarized: 1-indexed, CANL to pin 2, GND to pin 3, CANH to pin 7.

---

## Setting up the CAN Transceiver

For the Jetson, the CAN transceiver will need to be wired (for the final system when fdcanusb is not used). One should solder the CAN transceiver to cape J17 which is the optional CAN bus header. If one needs documentation, they should search up "Jetson Xavier NX Carrier Board Placement – Top View" in the "NVIDIA Jetson Xavier NX Developer Kit Carrier Board" Specification PDF.

One should also configure the CAN Transceiver in software. I have not done enough research but [this](https://elinux.org/Jetson/AGX_Xavier_CAN#CAN_Driver_Enable) resource may be useful.

---

## Connecting CAN-FD Lines

Not exactly sure what the best way to connect the CAN lines in our system yet. Maybe daisy chaining everything is sufficient?

## Limit Switches

The moteus has GPIO digital inputs (aux 1 and aux 2). This code currently has the outline for using limit switches, but it might be more complicated than expected.

One thing to know is whether to set the input as pull up or pull down depending on whether we intend for it to be active low or active high. See [here](https://github.com/mjbots/moteus/blob/main/docs/reference.md#pin-configuration) for more information on how to configure the pins.

One issue is that if we can check prevent velocity commands by checking the sign of velocity. However, this is not valid for position control since the velocity is still 0. One solution is to compare the current position with the requested position. Another solution is explained below.

The other solution is using the moteus library. The moteus allows one to set position_min and position_max, but that only stops movement for position control mode (unless velocity control mode is considered a position control mode?).