# PIAPA
Repository for the control of a Positionally and Inertially Aware Pick-and-place Architecture for industrial robots.

### How to use it:
Most of the functionality was application-specific, however the piapa.py `Rover` class could be useful as a framework for robot  movement.

    from piapa import Rover
    r = Rover() #create a Rover instance
    r.calibrateMag() #calibrate IMU magnetometer
    r.forw(0.5) #move forward 0.5m
    r.backw(0.5) #move backwards 0.5m
    r.pick() #use mechanical arm to pick objects
    r.place() #use mechanical arm to place objects
    print(r.hasObject) #returns true if arm has an object, false otherwise
    r.turn(30) #turn 30 degrees counterclockwise (no feedback)
    r.turnWithAngle(30) #turn 30 degrees counterclockwise (using IMU sensor for angle correction)
    print(r.readAngle()) #returns measurement from IMU magnetometer sensor
    print(r.avgAngle()) #returns average of 8 IMU magnetometer sensor readings
    print(r.readSonic()) #returns measurement from HC-SR04 ultrasonic sensor

If the library is meant to be used in a similar application as mine, the `Map` class could be used in conjuction to it, please
refer to `drodriguezg@uninorte.edu.co` for further support in accomodation to application.