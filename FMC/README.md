## FMC [Flight Management Computer]

Store data in SD Card and Flash Chip

*Stage 1*-> Self Test - generate zero values for all (zero maybe 0.5 or so due to precision errors)

*Stage 2*
Liftoff:
-> del(upward_axis)>buffer
-> ensure Accel. in y axis post n milliseconds, and not confirm liftoff

*Stage 3*
checks for apogee => Wait for freefall (IMU, BMP, Altimeter)
IMU for acc on x,y,z and o(angle) gyro data - 6 deg of freed: pitch, yaw, roll

Apogee by
1) Accel. in y axis
2) Ensure Accel. in y axis post n milliseconds, and not confirm apogee
3) Accel.<10ms-1 on x,y,z axes
4) If del(Altitude) is -ve
Known: 15.87 seconds from liftoff is expected apogee.
Override Apogee Logic: If t>=15, assume apogee

*Stage 4*
Recovery will be initiated by sending a digital high signal to one of the Arduino pins -> eject the parachute using linear actuator
-> Data Collection - BMP (Pressure, Altitude), IMU (Pitch, Yaw, Roll), Time - will continue until touchdown

Touchdown -> del(any axis accel.) = 0
Known: 287s is time to touchdown
