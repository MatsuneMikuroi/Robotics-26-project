# Robotics Project 2026

## Idea

Block's storing robots:

- A robot go search the block and bring them into a transition area
- A second robot take the block in the transition area and sort them by color:
  - If the color has its own storing area, bring the block there
  - Else, bring the block into the "trash"

## Material

Material needed:

- 2 e-puck
- 2 3d-printed support to grab the block
- Numerous blocks of different colors (including at least red, green, blue, black, yellow (or another color not recognize by the robot))

## Implementation

The first robot go grab a block in a free area, then bring it to the transition area (marked on the floor with a shade of gray), it then sends a message to the second robot (or the second robot uses its camera to detect if the block is in the transition area).
The second robot come grab the block, identify its color and sort it. To do it, it follows a line (black) and some crossroad (marked by a shade of gray) indicate to it which cross the epuck is (we can count the gray lines crossed to know the position of the robot). If the color isn't recognize by the robot, then it put into the "garbage" (special storing area away from the 4 others).
