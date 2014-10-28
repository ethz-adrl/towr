#(X++) - Dynamic Motion Planner

With this library one can specify a set of steps for a quadruped robot to 
execute and a trajectory for the Center of Gravity (CoG) will be generated. 
This trajectory enusures, that the Zero Moment Point(ZMP) is always inside the
current support triangle, defined by the given footholds. This keeps the robot
_dynamically_ stable during the walking gait.

**Tip**: After cloning the repository, generate [Doxygen](http://www.stack.nl/~dimitri/doxygen/) 
documentation for an example and step-by-step instructions how to use the code.

For further questions contact <winklera@ethz.ch>. 

------------------------------------------------------------------------------------
### Dependencies 

- Eigen

        sudo apt-get install libeigen3-dev

- Apache logging library log4cxx

        sudo apt-get install liblog4cxx10 && sudo apt-get install liblog4cxx10-dev
    
- (Optional) Google unit test framework

        # download headers and sources
        sudo apt-get install libgtest-dev 
              
        # build library     
        cd /usr/src/gtest             
        sudo cmake CMakeLists.txt
        sudo make
        
        # copy libgtest.a and libgtest_main.a to your /usr/lib folder
        sudo cp *.a /usr/lib
          
          
### Install 
- Once dependencies are installed, go to the top-level directory of xpp and type the
following commands:

        mkdir build
        cd build
        cmake ..    
        make
    
- To install the library `xpp` and header files globally in `/usr/local` run

        sudo make install    


### Documentation
- Generate Doxygen documentation from this code

        cd doc
        doxygen Doxyfile
        firefox html/index.html

    