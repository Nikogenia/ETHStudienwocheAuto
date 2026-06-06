# ETH Studienwoche Auto
source code of one solution for the autonomous line-following car challenge at the ETH Zürich Studienwoche 2026
of the department of electrical engineering and information technology (D-ITET)

A maze should be solved by following lines on the ground. The car should be able to detect intersections and make decisions on which way to turn. Sensor input is provided by 6 photosensitive diodes, amplified by a custom pcb circuit and read by an Arduino Nano. The car is powered by a battery and uses 4 DC motors for movement. The code implements a state machine to control the behavior of the car based on sensor input and timing.

The state machine can be visualized as follows:

```text
+--------------------------------------------------+
|                    FOLLOW_LINE                   |
+--------------------------------------------------+
| - Read sensors                                  |
| - Compute line error                            |
| - PD steering                                   |
| - Check for intersections                       |
+--------------------------------------------------+
      |                         |
      | Intersection detected   | All-black held
      | or all-white lost line  | > BLACK_HOLD_TIME
      v                         v
+----------------------+   +----------------------+
|     INTERSECTION     |   |       STOPPED        |
+----------------------+   +----------------------+
| - Center on junction |   | Motors OFF           |
| - Detect exits       |   | Terminal state       |
| - Build Junction     |   +----------------------+
+----------------------+
           |
           | after INTERSECTION_CENTER_TIME
           v
   +------------------------------+
   | decideIntersection()         |
   +------------------------------+
      |            |            |
      | Left       | Forward    | Right
      v            v            v
+-------------+    |    +-------------+
|SEARCH_LINE  |    |    |SEARCH_LINE  |
|    _LEFT    |    |    |   _RIGHT    |
+-------------+    |    +-------------+
| Rotate left |    |    | Rotate right|
+-------------+    |    +-------------+
      |            |            |
      | timeout / recovery      |
      +------------+------------+
                   |
                   v
        +-----------------------+
        | SEARCH_LINE_RECOVERY  |
        +-----------------------+
        | Reverse briefly       |
        | Set lockout timer     |
        +-----------------------+
                   |
                   v
        +-----------------------+
        |     FOLLOW_LINE       |
        +-----------------------+


LEFT-HAND DECISION LOGIC
========================

INTERSECTION
    |
    +-- left available?
    |       |
    |       +-- YES --> SEARCH_LINE_LEFT
    |
    +-- forward available?
    |       |
    |       +-- YES --> FOLLOW_LINE
    |
    +-- right available?
    |       |
    |       +-- YES --> SEARCH_LINE_RIGHT
    |
    +-- otherwise
            |
            +--> SEARCH_LINE_LEFT
                 (full search / U-turn allowed)


STATE TRANSITIONS
=================

FOLLOW_LINE
 ├─ intersection detected ----------> INTERSECTION
 ├─ all sensors white --------------> INTERSECTION
 └─ all sensors black long enough --> STOPPED

INTERSECTION
 ├─ left branch chosen -------------> SEARCH_LINE_LEFT
 ├─ right branch chosen ------------> SEARCH_LINE_RIGHT
 └─ forward branch chosen ----------> FOLLOW_LINE

SEARCH_LINE_LEFT
 ├─ line found ---------------------> FOLLOW_LINE
 └─ search timeout -----------------> SEARCH_LINE_RECOVERY

SEARCH_LINE_RIGHT
 ├─ line found ---------------------> FOLLOW_LINE
 └─ search timeout -----------------> SEARCH_LINE_RECOVERY

SEARCH_LINE_RECOVERY
 └─ recovery complete -------------> FOLLOW_LINE

STOPPED
 └─ no exits (terminal state)
```
