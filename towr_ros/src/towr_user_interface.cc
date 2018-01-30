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

#include <towr_ros/towr_user_interface.h>

#include <ncurses.h>

#include <xpp_states/convert.h>

#include <towr_ros/TowrCommand.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/height_map_examples.h>


namespace towr {

TowrUserInterface::TowrUserInterface ()
{
  printw("************************************************************\n");
  printw("              TOWR user interface (v1.1.0) \n");
  printw("                \u00a9 Alexander W. Winkler \n");
  printw("            https://github.com/ethz-adrl/towr\n");
  printw("************************************************************\n\n");

  PrintHelp();

  ::ros::NodeHandle n;

  user_command_pub_ = n.advertise<towr_ros::TowrCommand>(towr_msgs::user_command, 1);
  printw("Publishing to: %s\n\n", user_command_pub_.getTopic().c_str());

  // publish goal zero initially
  goal_geom_.lin.p_.setZero();
  goal_geom_.lin.p_ << 1.0, 0.0, 0.46;
  goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''

  terrain_id_    = 0;
  gait_combo_id_ = 3;
  total_duration_ = 2.0;
  replay_trajectory_ = false;
  use_solver_snopt_ = false;
  optimize_ = false;
  publish_optimized_trajectory_ = false;
}

void
TowrUserInterface::PrintHelp() const
{
  printw("\n");
  printw("The keyboard mappings are as follows:\n\n");
  printw("h            \t display this help\n");
  printw("arrow keys   \t move goal position in xy-plane\n");
  printw("page up/down \t modify goal height\n");
  printw("keypad       \t modify goal orientation\n");
  printw("g            \t change gait\n");
  printw("t            \t change terrain\n");
  printw("o            \t optimize motion\n");
  printw("r            \t replay motion\n");
  printw("+/-          \t increase/decrease total duration\n");
  printw("s            \t toggle solver between IPOPT and SNOPT\n");
  printw("q            \t close user interface\n");
  printw("\n");
}

void
TowrUserInterface::CallbackKey (int c)
{
  const static double d_lin = 0.1;  // [m]
  const static double d_ang = 0.25; // [rad]

  switch (c) {
    case 'h':
      PrintHelp();
      break;
    case 'q':
      printw("Closing user interface\n");
      ros::shutdown(); break;
    case KEY_RIGHT:
      goal_geom_.lin.p_.x() -= d_lin;
      PrintVector(goal_geom_.lin.p_);
      break;
    case KEY_LEFT:
      goal_geom_.lin.p_.x() += d_lin;
      PrintVector(goal_geom_.lin.p_);
      break;
    case KEY_DOWN:
      goal_geom_.lin.p_.y() += d_lin;
      PrintVector(goal_geom_.lin.p_);
      break;
    case KEY_UP:
      goal_geom_.lin.p_.y() -= d_lin;
      PrintVector(goal_geom_.lin.p_);
      break;
    case KEY_PPAGE:
      goal_geom_.lin.p_.z() += 0.5*d_lin;
      PrintVector(goal_geom_.lin.p_);
      break;
    case KEY_NPAGE:
      goal_geom_.lin.p_.z() -= 0.5*d_lin;
      PrintVector(goal_geom_.lin.p_);
      break;

    // desired goal orientations
    case '4':
      goal_geom_.ang.p_.x() -= d_ang; // roll-
      break;
    case '6':
      goal_geom_.ang.p_.x() += d_ang; // roll+
      break;
    case '8':
      goal_geom_.ang.p_.y() += d_ang; // pitch+
      break;
    case '2':
      goal_geom_.ang.p_.y() -= d_ang; // pitch-
      break;
    case '1':
      goal_geom_.ang.p_.z() += d_ang; // yaw+
      break;
    case '9':
      goal_geom_.ang.p_.z() -= d_ang; // yaw-
      break;

    // terrains
    case 't':
      terrain_id_ = AdvanceCircularBuffer(terrain_id_, towr::K_TERRAIN_COUNT-1);
      printw("Switched terrain to %i \n", terrain_id_);
      break;

    case 'g':
      gait_combo_id_ = AdvanceCircularBuffer(gait_combo_id_, kMaxNumGaits_);
      printw("Switched gait to combo %s \n", std::to_string(gait_combo_id_).c_str());
      break;

    // speed
    case '+':
      total_duration_ += 0.2;
      printw("Total duration increased to %f \n", total_duration_);
    break;
    case '-':
      total_duration_ -= 0.2;
      printw("Total duration decreased to %f \n", total_duration_);
    break;


    case 'o':
      printw("Optimize motion request sent\n");
      optimize_ = true;
      break;
    case 'p':
        publish_optimized_trajectory_ = true;
        printw("Publish optimized trajectory request sent\n");
      break;
    case 's':
      printw("Toggled NLP solver type\n");
      use_solver_snopt_ = !use_solver_snopt_;
      break;
    case 'r':
      printw("Replaying already optimized trajectory\n");
      replay_trajectory_ = true;
      break;
    default:
      break;
  }

  PublishCommand();
}

void TowrUserInterface::PublishCommand()
{
  towr_ros::TowrCommand msg;
  msg.goal_lin          = xpp::Convert::ToRos(goal_geom_.lin);
  msg.goal_ang          = xpp::Convert::ToRos(goal_geom_.ang);
  msg.replay_trajectory = replay_trajectory_;
  msg.use_solver_snopt  = use_solver_snopt_;
  msg.optimize          = optimize_;
  msg.terrain_id        = terrain_id_;
  msg.gait_id           = gait_combo_id_;
  msg.total_duration    = total_duration_;
  msg.publish_traj      = publish_optimized_trajectory_;

  user_command_pub_.publish(msg);

  optimize_ = false;
  replay_trajectory_  = false;
  publish_optimized_trajectory_ = false;
}

int TowrUserInterface::AdvanceCircularBuffer(int& curr, int max) const
{
  return curr==max? 0 : curr+1;
}

void
TowrUserInterface::PrintVector(const Eigen::Vector3d& v) const
{
  printw("Goal position set to %f %f %f \n",
         goal_geom_.lin.p_.x(),
         goal_geom_.lin.p_.y(),
         goal_geom_.lin.p_.z()
         );
}


} /* namespace towr */


// the actual ros node
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "useri_iterface_node");

  initscr();
  cbreak();              // disables buffering of types characters
  noecho();              // suppresses automatic output of typed characters
  keypad(stdscr, TRUE);  // to capture special keypad characters
  scrollok(stdscr,TRUE); // so new lines are printed also below terminal

  towr::TowrUserInterface keyboard_user_interface;

  while (ros::ok())
  {
    int c = getch(); // call your non-blocking input function
    keyboard_user_interface.CallbackKey(c);
    refresh();
  }

  endwin();

  return 1;
}

