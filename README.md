# XPP Optimizer

The optimizer (NLP, QP) for foothold positions and body trajectory for a quadruped robot.

------------------------------------------------------------------------------------
### Dependencies 

- Eigen

        $ sudo apt-get install libeigen3-dev
          
- Ipopt/Snopt. For this it's best to see the appropriate repos:

         $ git clone https://bitbucket.org/adrlab/ipopt.git

- xpp_common

         $ git clone git@bitbucket.org:adrlab/xpp_common.git

- (Optional) Google unit test framework

        $ sudo apt-get install libgtest-dev       # download headers and sources                  
        $ cd /usr/src/gtest             
        $ sudo cmake CMakeLists.txt
        $ sudo make                               # build library        
        $ sudo cp *.a /usr/lib                    # copy libgtest.a and libgtest_main.a to your /usr/lib folder

### Configuration ###
You must set the environmental variable 'IPOPT_DIR' to point to the Ipopt Installation.

    $ export IPOPT_DIR=~/3rd_party_software/ipopt

### Contact 
Alexander W. Winkler (winklera@ethz.ch)