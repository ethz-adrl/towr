
#include <iostream>
#include <string>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>

#include <xpp_msgs/RobotStateCartesian.h>


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
