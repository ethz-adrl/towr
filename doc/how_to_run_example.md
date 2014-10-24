Set-Up and Run Example
===============================================

This code explains how to use the xpp library to create a CoG trajectory for
variable footsteps. 

An implementation example can be found in example.cc, which is compiled
when building the library. To run the binary the global variable `XPP_ROOT`
must be set. Navigate to the top level directory `xpp` and run: 
    
    source test/set_env_variables.sh
    ./bin/example

This example prints out the state of the trajectory at specific times `t`.
For a detailed explanation of this code, refer to 
[this](_exp.html) explanation.

Additionally the optimized spline coefficients are saved in `test/matlab/trajectory_coefficients`.
To visualize the coefficients, open matlab and run the matlab script

    test/matlab/display_trajectory.m
    

    
