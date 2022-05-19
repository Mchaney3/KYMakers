***   IMPORTANT
Check my wires for left motor. It spins in one direction but not the other 
indicating either the code isn;t setting IN4 LOW or HIGH or the wire is bad

DIRECTION  INPUT 1 INPUT 2 INPUT 3 INPUT 4
Forward 0 1 0 1
Backward  1 0 1 0
Right 0 1 0 0
Left  0 0 0 1
Stop  0 0 0 0

With my robot build, My "Right" motor is facing from the rear, which means the left motor runs in the opposite direction. 
This is my truth chart for Motor functionality

                                    Single Track Steer
                        Right Motor                     Left Motor
DIRECTION         INPUT 1         INPUT 2         INPUT 3         INPUT 4
Forward             0               1               1               0
Backward            1               0               0               1
Right               0               0               1               0
Left                0               1               0               0
Stop                0               0               0               0




                                    Dual Track Steer
                        Right Motor                     Left Motor
DIRECTION         INPUT 1         INPUT 2         INPUT 3         INPUT 4
Forward             0               1               1               0
Backward            1               0               0               1
Right               1               0               1               0
Left                0               1               0               1
Stop                0               0               0               0
