# ETH Studienwoche Auto
source code of one solution for the autonomous line-following car challenge at the ETH Studienwoche 2026
of the department of electrical engineering and information technology (D-ITET)

A maze should be solved by following lines on the ground. The car should be able to detect intersections and make decisions on which way to turn. Sensor input is provided by 6 photosensitive diodes, amplified by a custom pcb circuit and read by an Arduino Nano. The car is powered by a battery and uses 4 DC motors for movement. The code implements a state machine to control the behavior of the car based on sensor input and timing.

The state machine can be visualized as follows:


