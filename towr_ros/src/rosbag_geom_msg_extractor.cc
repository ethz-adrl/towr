/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>

#include <xpp_msgs/RobotStateCartesian.h>

/**
 * Extracts the standard ROS geometry_msgs/Vector3.h from a ROS bag of
 * RobotStateCartesian and writes them to a new bag. The bags with standard
 * messages can then easily be imported and plotted in matlab.
 *
 * See towr/matlab/plot_rosbag.m for an example of how to open these.
 */
int main(int argc, char *argv[])
{
  if (argc==1) {
    std::cerr << "Error: Please enter path to bag file\n";
    return 0;
  }

  std::string bag_file = argv[1];

  rosbag::Bag bag_r;
  bag_r.open(bag_file, rosbag::bagmode::Read);
  std::cout << "Reading from bag " + bag_r.getFileName() << std::endl;

  // select which iterations (message topics) to be included in bag file
  std::string topic = "/xpp/state_des";
  rosbag::View view(bag_r, rosbag::TopicQuery(topic));
  if (view.size() == 0) {
    std::cerr << "Error: Topic " << topic << " doesn't exist\n";
    return 0;
  }

  // write the message with modified timestamp into new bag file
  rosbag::Bag bag_w;
  bag_w.open("/home/winklera/Desktop/matlab_rdy.bag", rosbag::bagmode::Write);

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    ros::Time t = m.getTime();
    auto state_msg = m.instantiate<xpp_msgs::RobotStateCartesian>();
    bag_w.write("base_pose", t, state_msg->base.pose);
    bag_w.write("base_acc", t, state_msg->base.accel.linear);

    int n_feet = state_msg->ee_motion.size();

    for (int i=0; i<n_feet; ++i) {
      bag_w.write("foot_pos_"+std::to_string(i), t, state_msg->ee_motion.at(i).pos);
      bag_w.write("foot_force_"+std::to_string(i), t, state_msg->ee_forces.at(i));
    }
  }

  bag_r.close();
  std::cout << "Successfully created bag " + bag_w.getFileName() << std::endl;
  bag_w.close();
}
