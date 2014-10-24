#(X++) - Dynamic Motion Planner Documentation

With this library one can specify a set of steps for a quadruped robot to 
execute and a trajectory for the Center of Gravity (CoG) will be generated. 
This trajectory enusures, that the Zero Moment Point(ZMP) is always inside the
current support triangle, defined by the given footholds. This keeps the robot
_dynamically_ stable during the walking gait.

An example of what to do with this code can be found [here](md_how_to_run_example.html).

For further questions contact <winklera@ethz.ch>. 

### Dependencies 
- Apache logging library log4cxx

        sudo apt-get install liblog4cxx10 && sudo apt-get install liblog4cxx10-dev
    
- (Optional) Google unit test framework

        sudo apt-get install libgtest-dev 
          
          
### Install 
- Once dependencies are installed, go to the top-level directory of xpp and type the
following commands:

        mkdir build
        cd build
        cmake ..    
        make
    
- To install the library and header files globally (`/usr/local`) run

        sudo make install    


### Documentation
- Generate Doxygen documentation from this code

        cd ./doc
        doxygen Doxyfile
    
- For an example of how to use this code, start [here](./test/md_how_to_run_example.html)