# Task logic

robot initiates (all red LEDs on while sync)

robot searches wall (all RGB LEDs on blue while searching)

robot finds wall and start following it on its left side (LED 2 on, others off)

robot ends when it has completed to map the wall (yaw $\geq \ldots^{\circ}$)

robot searches for green block (all RGB LEDs on flash green while searching)

robot finds green block and moves towards it (LED 3  green, others off)

robot go back to the other side of the wall (get the shortest path to the other side of the wall)

robot searches for red block (all RGB LEDs on flash red while searching)

robot finds red block and moves towards it (LED 3 red, others off)

compute the shortest path to a wall extremity and to the green block, and move to the green block (LED 3 green - red, others off)

robot ends when it has reached the green block (LEDs 0, 2, 4, 6 on, others off)
