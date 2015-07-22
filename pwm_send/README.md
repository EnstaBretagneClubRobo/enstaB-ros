# ros-rc_override
Using mavros, the package publish a service to override rc_in of APM

We haven't manage to make the rc_override working (it does publish but after modification on ardupilot
it still doesn't work) 
So we add a package (pwm_serial_py) to write PWM from usb serial link we the board Polulu Maestro and SSC-32U,
then with a cytron board we change channel between the PC (ROS) and the remote controller.
