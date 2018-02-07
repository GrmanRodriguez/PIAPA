# PIAPA Robot

## Logbook

### Grman Rodriguez, Edwin Acosta

---

**Sept 14th, 2017:**

The Raspberry Pi was connected to the Rover through the L298 drivers. raspMots.py (commit: ba23b466729ac75949227a090a294037d93cc3b6) was ran. The Rover's wheels moved accordingly as seen on [here](https://youtu.be/SFIwYYLxcLU). However, the motors must be marked to know which pin makes their respective wheel move forward, as wheel motion was unclear. Next step is making the system battery-powered and take care of cable management. 

---

**Sept 15th, 2017:**

The motors have been properly marked and now the robot has a sense of forward. The pins that make each wheel move forward have been properly identified as well.

---

**Oct 9th, 2017:**

The Lynxmotion AL5C robotic arm has been characterized. Now it can be used with full knowledge on how to move it.

---

**Oct 26th, 2017:**:

The rover vehicle is now battery powered and its angle movement has been calibrated. raspMots.py (commit: b47cf07513fc8e78cc15ccb1650d6be2a2ee6a39) was imported as a library and some of its commands working are shown [here](https://youtu.be/wpJkDJAA3jc).