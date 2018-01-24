
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
