# MDP-STM32

## Commands:
### Straight line movement:
#### FW/BW: Forward/backward
- FW--/BW--: manual mode
- FW+ dist/BW+dist: move given distance

#### DT:move until a specified distance from the obstacle
- DT + dist

#### FL/FR/BL/BR: Forward left/right, backward left/right
- FL/FR/BL/BR 00: low speed, move 4cm -> turn 90 degrees -> move 7cm
- FL/FR/BL/BR 20: medium speed, move 4cm -> turn 90 degrees -> move 7cm
- FL/FR/BL/BR 30: high speed, turn 90 degrees -> move 4cm

#### FA/FC/BA/BC: forward/backward clockwise/anti-clockwise rotation with specified angle

