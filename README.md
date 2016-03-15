# Segway
This repository will hold code for robots balancing on two wheels, made with various robotics platforms.

This fork uses python 2.7 and ev3dev unlike the original from Laurens. It is more legible, but slightly slower: the control
loop runs every 17ms instead of 10ms.

## Currently available platforms

LEGO MINDSTORMS EV3 (ev3dev/Python):

- [Building instructions] (http://robotsquare.com/2014/06/23/tutorial-building-balanc3r/)
- Add a Touch Sensor to Port 1. I added it just like the Gyro, but on the other side of the brick. This will be the program's safe stop button.


## Work in progress

Optimize ev3dev-lang-python for read/write speed.