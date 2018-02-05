# Arm Control Commands

The PIAPA system includes an arm that allows it to grab the desired objects.

For this prototype, the [Lynxmotion AL5C](https://www.researchgate.net/publication/47697116/figure/fig2/AS:320350953984001@1453389027027/AL5C-arm-from-Lynxmotion-Inc-8.png) was used. Said mechanism's servos are controlled by an [SSC-32 Controller](https://www.robotshop.com/media/files/pdf2/lynxmotion_ssc-32u_usb_user_guide.pdf). This enables control of the servos through serial commands of position, velocity and/or time of execution. Below is a translation of the basic commands to move each servo to the respective angle they leave the part of the arm they control in.

**Motor #0**

Controls the rotation of the whole arm.

*Neutral Position (Motor #1 sits between the 2 ID stickers):* p1600 

*90° to the left:* p950

*90° to the right:* p2550

**Motor #1**

Controls the lift angle of the upper arm.

*Neutral Position (Upper arm is lifted 90° relative to the ground):* p1850

*90° forward:* p400

**Motor #2**

Controls the lift angle of the forearm.

*Neutral Position (Forearm is aligned with upper arm):* p700

*90° forward:* p1450

**Motor #3**

Controls the lift angle of the wrist.

*Neutral Position (Wrist is aligned with forearm):* p2100

*90° backwards:* p1200

**Motor #4**

Controls the grip.

*Fully open:* p900

*Fully closed:* p2000
