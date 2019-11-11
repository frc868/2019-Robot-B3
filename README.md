# 2019-Robot-B3

The code for the robot that will be presumably used at Boiler Bot Battle '19.
This has been refactored from 2019-Robot to utilize TimedRobot over command-based.

## Important notes/quirks
* `ControllerWrapper` and `ButtonWrapper` represent reimplementations of the former functions provided by `Button`
  * To map a button, you can reference the `bA` (read: button A) or `dN` (read: d-pad north) objects under `ControllerWrapper`
  * Map buttons using lambdas (anonymous functions): `() -> Robot.manipulator.toggle()`
  * `Helper.deadzone()` should pretty much never be used if you're referencing a `ControllerWrapper`, that functionality's built in
* `TurnToAngleGyro` probably will never be tuned, so don't use it
* `Manipulator` references both the ball and hatch intakes

## Conventions
* All [coding standards](https://techhounds.com/TechHOUNDS%20Coding%20Standards.pdf) apply!
* When creating a subsystem, implement most subsystem-related functionality within it. Seperate classes are discouraged but sometimes necessary (see `TurnToAngleGyro`)
  * If there's a PID, make it part of the subsystem
  * Any subsystems with periodic methods should be handled with an `update()` method called in `robotPeriodic()`
* Use `enum`s when representing multi-state systems (see `ElevatorPosition`) and `boolean`s when representing two-state systems (see tilt brake)
  * Please don't use multiple booleans to represent the same system
* Some sensors should be singletons (see `Camera`) and some shouldn't (see `IRLimit`) -- use your own discretion