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

The first robot go grab a block in a free area, then bring it to the transition area (marked on the floor with a shade of gray), it then sends a message to the second robot.
The second robot come grab the block, identify its color and sort it
