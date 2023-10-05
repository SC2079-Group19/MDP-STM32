# MDP-STM32

## Command
5 characters for each command

### Straight line movement:
#### FW/BW: Forward/backward
- FW+ dist/BW+dist: move given distance
- FW--/BW--: manual mode
#### DT: move until a specified distance from the obstacle
- DT + dist

### Tuning:
#### FL/FR/BL/BR: Forward left/right, backward left/right - for A*
- FL/FR/BL/BR 00: low speed,
- FL/FR/BL/BR 20: medium speed
- FL/FR/BL/BR 30: high speed
- FL/FR/BL/BR --: manual mode
#### FA/FC/BA/BC: forward/backward clockwise/anti-clockwise rotation with specified angle  - for Dubin
#### TL/TR: turning left/right max

### Preemptive stop:
#### ST

### Reset turning angle:
#### RS
