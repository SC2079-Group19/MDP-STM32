# MDP-STM32

## About

This repository contains the STM32CubeIDE project for SC2079 Multi-Disciplinary Project (MDP).

## Commands

Each command consists of 5 characters.

### Straight-line movement:

#### FW/BW: Forward/backward

- `FW` + dist/`BW` + dist: move a given distance
- `FW`--/`BW`--: manual mode

#### DT: move until a specified distance from the obstacle

- `DT` + dist

#### (Experimental) DZ: move until a specified distance from the obstacle with a faster speed

### Turning:

#### FL/FR/BL/BR: Forward left/right, backward left/right - for A\* Algorithm

- `FL/FR/BL/BR` 00: for task 1 indoor arena
- `FL/FR/BL/BR` 30: for task 1 outdoor arena
- `FL/FR/BL/BR` --: manual mode

#### FA/FC/BA/BC: forward/backward clockwise/anti-clockwise rotation with a specified angle - for Dubin Algorithm

- `FA/FC/BA/BC` + target angle (in degrees)

#### TL/TR: turning left/right max

### Preemptive stop:

- `ST`

### Reset turning angle:

- `RS`

### Task-Specific Command (Task 2):

#### TA: Task 2 Turn A - 10x10 obstacle

- `TA` + 01 /02

#### AM: Avocado Milkshake - the turn following Turn A

- `AM` + 01 /02
  _We would like to express our gratitude to the creamy and delightful Avocado Milkshake from NIE canteen for keeping us fueled and motivated during the endless calibration and tuning of MDP tasks._

#### TB: Task 2 Turn B - 10x(variable length) obstacle

- `TB` + 01 /02

#### GH: Task 2 Go Home command

- `GH` + 01 /02

## Acknowledgement:

[CZ3004-Group-28/STM32_workspace](https://github.com/CZ3004-Group-28/STM32_workspace) by [wxchee (Wei Xian)](https://github.com/wxchee)
