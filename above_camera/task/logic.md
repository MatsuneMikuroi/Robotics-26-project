Wall search (loverProx)
+-------------------------------------------------------------------------------+
|                                                                               |
|                                                                               |
|                                                                               |
|    rrrr                                                                       |
|    rrrr                            w                                          |
|    rrrr                            w                                          |
|                                    w                                          |
|                    O ->    ->      w                                          |
|                                    w                                          |
|                                    w                                          |
|                                    w                            gggg          |
|                                    w                            gggg          |
|                                                                 gggg          |
|                                                                               |
|                                                                               |
|                                                                               |
+-------------------------------------------------------------------------------+
next state when abs(SLL.getAverage() - TH_PROX) < PROX_EPS



Wall following (wall_following + PID)
+-------------------------------------------------------------------------------+
|                                                                               |
|                                                                               |
|                                                                               |
|    rrrr                           ->                                          |
|    rrrr                            w                                          |
|    rrrr                            w                                          |
|                                  ^ w                                          |
|                    O             | w |                                        |
|                                    w v                                        |
|                                    w                                          |
|                                    w                            gggg          |
|                                    w                            gggg          |
|                                    <-                           gggg          |
|                                                                               |
|                                                                               |
|                                                                               |
+-------------------------------------------------------------------------------+
next state when ...



Go to y_max/y_min (depending on the direction of wall following) and turn toward the green block area (???)

+-------------------------------------------------------------------------------+
|                                ->                                             |
|                                                                               |
|                                                                               |
|    rrrr                                                                       |
|    rrrr                        ^   w                                          |
|    rrrr                        |   w                                          |
|                                    w                                          |
|                    O               w                                          |
|                                    w                                          |
|                                ^   w                                          |
|                                |   w                            gggg          |
|                                    w                            gggg          |
|                                                                 gggg          |
|                                                                               |
|                                                                               |
|                                                                               |
+-------------------------------------------------------------------------------+
next state when ty > y_max - EPS_DIST and ...




Go in the green block area and search for the green block (scan + loverColor)
+-------------------------------------------------------------------------------+
|                                ->    ->    -> ...                             |
|                                                                               |
|                                        |  ^                                   |
|    rrrr                                v  |                                   |
|    rrrr                            w                                          |
|    rrrr                            w                                          |
|                                    w                                          |
|                    O               w                                          |
|                                    w   |  ^                                   |
|                                    w   v  |                                   |
|                                    w                            gggg          |
|                                    w                            gggg          |
|                                                                 gggg          |
|                                        |  ^                                   |
|                                        v  |                                   |
|                                         ->                                    |
+-------------------------------------------------------------------------------+
next state when loverColor("green") < (EPS_???, EPS_???)




Go from the green block to the origin (compute path to avoid the wall and go to the origin following this path) (???)
+-------------------------------------------------------------------------------+
|                                                                               |
|                                                                               |
|                                                                               |
|    rrrr                                                                       |
|    rrrr                            w                                          |
|    rrrr                            w                                          |
|                                    w                                          |
|                    O               w                                          |
|                                    w                                          |
|                                    w                                          |
|                        ^           w                            gggg          |
|                        |           w                         <- gggg          |
|                                                                 gggg          |
|                                                                               |
|                                   <-                                          |
|                                                                               |
+-------------------------------------------------------------------------------+
next state when abs(tx) < EPS_DIST and abs(ty) < EPS_DIST



from the origin, scan the area to find the red block (scan + loverColor)
+-------------------------------------------------------------------------------+
|                  <-                                                           |
|                                                                               |
|                                                                               |
|    rrrr                                                                       |
|    rrrr        |   ^               w                                          |
|    rrrr        v   |               w                                          |
|                                    w                                          |
|                    O               w                                          |
|                                    w                                          |
|                                    w                                          |
|                                    w                            gggg          |
|                |                   w                            gggg          |
|                v                                                gggg          |
|                                                                               |
|                                                                               |
|          ... <-                                                               |
+-------------------------------------------------------------------------------+
next state when loverColor("red") < (EPS_???, EPS_???)




from the red block, go to the green block (compute path to avoid the wall and go to the green block following this path) (???)

+-------------------------------------------------------------------------------+
|                                                                               |
|                                                                               |
|                                                                               |
|    rrrr                                                                       |
|    rrrr                            w                                          |
|    rrrr                            w                                          |
|                                    w                                          |
|                    O               w                                          |
|                                    w                                          |
|                                    w                                          |
|                                    w                            gggg          |
|                                    w                            gggg          |
|                                                                 gggg          |
|                                                                               |
|                                                                               |
|                                                                               |
+-------------------------------------------------------------------------------+
next state when loverColor("green") < (EPS_???, EPS_???)

task completed