# ROMI-Learning
ROMI Robot Learning Base

# Walk through up to FORWARD command.  
## A forward only drive chassis/driveTrain

My ROMI doesn't drive straight.  So what do I do next?
Instead of using arcadeDrive, switch to tankDrive(leftSpeed, rightSpeed) and add a speed modifier based on sample runs and observing leftEncoder and rightEncoder outputs.  A little back and forth until get the right modifier so it can drive 'straight' for 'short distance' (i.e. 10in).

This will mean that when we get to the TURN, would have to do something 'similar'.

Code as of 2/16/2021 shows the modifiers (which will needed to be tweated periodically).  Question: how to avoid doing hard coded values?  => Added in calibration mode which then stored the 'modifiers' somewhere. 

The somewhere can be 'hard coded' saved value/memory locations, SQLite database, etc.  Options to consider...

