# MDP-STM32

## Command
5 charactres
### Straight line movement:
#### FW/BW: Forward/backward
- FW+ dist/BW+dist: move given distance
- FW--/BW--: manual mode
#### DT:move until a specified distance from the obstacle
- DT + dist

### Tuning:
#### FL/FR/BL/BR: Forward left/right, backward left/right
- FL/FR/BL/BR 00: low speed, move 4cm -> turn 90 degrees -> move 7cm
- FL/FR/BL/BR 20: medium speed, move 4cm -> turn 90 degrees -> move 7cm
- FL/FR/BL/BR 30: high speed, turn 90 degrees -> move 4cm
- FL/FR/BL/BR --: manual mode
#### FA/FC/BA/BC: forward/backward clockwise/anti-clockwise rotation with specified angle
#### TL/TR: turning left/right max

### Preempt stop:
#### ST

### Reset turning angle:
#### RS
