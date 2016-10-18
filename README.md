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

        $ sudo apt-get install libeigen3-dev
    
- (Optional) Google unit test framework

        $ sudo apt-get install libgtest-dev       # download headers and sources                  
        $ cd /usr/src/gtest             
        $ sudo cmake CMakeLists.txt
        $ sudo make                               # build library        
        $ sudo cp *.a /usr/lib                    # copy libgtest.a and libgtest_main.a to your /usr/lib folder
          
- Ipopt/Snopt. For this it's best to see the appropriate repos:

         $ git clone https://bitbucket.org/adrlab/ipopt.git


### Documentation
- Doxygen version >1.8.6 required
- Generate Doxygen documentation from this code

        $ cd doc
        $ doxygen Doxyfile
        $ google-chrome  html/index.html

- A powerpoint presentation if available in the "Downloads" section of this repository.