# backpackArmControl
Controls the backpack arm. As modular as possible! Only works with Hebi modules for now.
Main functions at top level, others in folders.
mainStartup is the most recent "main" file.

Starts out by loading a file with design parameters and joint configurations.
Uses Hebi API for trajectory generation and groups.
Use a vrjoystick to move the arm around, with a variety of modes.
Modes include:
- gravity compensation, commands torques to balance gravity. Can teach the arm new poses by moving it by hand.
- hold the position.
- move between preset poses
- push with impedance controller
- IK end effector control
- follow a persons hand with Leap motion sensor (experimental)

The setup folder contains scripts to make the robot command groups and structures,
and set the gains. 

The folder "two arms" and "voice input" are experimental and not really used.

There are not many safety or collision checks so be careful not to hit stuff.
For instance, the trajectories are all joint space interpolation which might collide.

TO DO
Scale trajectory speed by how much the most joint moves
Note: there may be a small issue with gravity direction,
  sometimes  -[fbk.accel] works better than [fbk.accel] for gravity direction.
I think that the accelerometer z might be flipped, that would account for it.