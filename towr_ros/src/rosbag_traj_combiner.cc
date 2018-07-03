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

#include <ros/init.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>

#include <xpp_msgs/topic_names.h>
#include <towr_ros/topic_names.h>

/**
 * Takes a ROS bag of optimization results (with intermediate iterations), and
 * strings them together so multiple iterations are played back sequentially.
 *
 * (used for RA-L submission video).
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rosbag_trajectory_combiner");

  std::string name = "/home/winklera/bags/optimal_traj";

  rosbag::Bag bag_r;
  bag_r.open(name+".bag", rosbag::bagmode::Read);
  ROS_INFO_STREAM("Reading from bag " + bag_r.getFileName());


  // get number of iterations in bag file
  int n_opt_iterations = 0;
  rosbag::View view1(bag_r, rosbag::TopicQuery(towr_msgs::nlp_iterations_count));
  BOOST_FOREACH(rosbag::MessageInstance const m, view1) {
    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    n_opt_iterations = i->data;
  }

  // select which iterations (message topics) to be included in bag file
  std::vector<std::string> topics;
  ROS_INFO_STREAM("Detected " + std::to_string(n_opt_iterations) + " iterations");
  int n_visualizations = 5; // total number of visualizations is fixed
  int frequency = std::floor(n_opt_iterations/n_visualizations);

  for (int i=0; i<n_visualizations; ++i)
    topics.push_back(towr_msgs::nlp_iterations_name + std::to_string(frequency*i));
  topics.push_back(towr_msgs::nlp_iterations_name + std::to_string(n_opt_iterations-1)); // for sure add final trajectory
  rosbag::View view(bag_r, rosbag::TopicQuery(topics));


  // change the timestamp so iterations are played back subsequently
  std::map<std::string, double> t_iter;
  double duration = view.getEndTime().toSec(); // duration of the trajectory
  for (int i=0; i<topics.size(); ++i)
    t_iter[topics.at(i)] = i*duration;

  ROS_INFO_STREAM("Visualizing messages:");
  for (auto m : t_iter)
    std::cout << m.first << std::endl;


  // write the message with modified timestamp into new bag file
  rosbag::Bag bag_w;
  bag_w.open(name + "_combined.bag", rosbag::bagmode::Write);

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    double t_global = t_iter.at(m.getTopic()) + m.getTime().toSec();
    bag_w.write(xpp_msgs::robot_state_desired, ::ros::Time(t_global), m);
  }

  bag_r.close();
  bag_w.close();
  ROS_INFO_STREAM("Successfully created bag " + bag_w.getFileName());

  return 1;
}
