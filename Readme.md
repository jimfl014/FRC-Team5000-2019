# FRC-Team5000-2017
This is the repo for the robotics code being developed by Team 5000 for the 2017 FIRST Robotics Competition

Tasks to Program

Drive for mechanum wheels
	-joystick(front, rear, side-to-side)
	-angled drive front & back, both left/right 45 degrees
	-autonomous for pre-placed gear

Gear(autonomous)
	-doors open and close at constant speed
	-sensor to communicate when peg is far enough beneath gear
	-reverse drive once sensor is triggered

Winch
	-start slow, speed up after catching rope
	-sensor communicates when the robot has contacted the top of the rope --> stop but do not reverse

Cameras
	-front/back, 360 degree turn w/ buttons

Notes about the Winch
-Code will control the motor
-By default it won't be doing anything
-Pressing one of the joystick buttons will have it move forward
-Pressing a different button will have it move backwards
-Pressing the button again will have it stop
-We might be using a sensor to stop it automatically, but it hasn't been integrated yet
-We want to put variables on the dashboard that allow us to independently control the speed of each direction