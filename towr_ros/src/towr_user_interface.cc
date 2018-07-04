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
#include <towr/terrain/examples/height_map_examples.h>


namespace towr {


enum YCursorRows {HEADING=6, OPTIMIZE=8, REPLAY, GOAL_POS, GOAL_ORI, GAIT, TERRAIN, DURATION, PUBLISH, CLOSE, END};
static constexpr int Y_STATUS      = END+1;
static constexpr int X_KEY         = 1;
static constexpr int X_DESCRIPTION = 10;
static constexpr int X_VALUE       = 35;


TowrUserInterface::TowrUserInterface ()
{
  printw(" ******************************************************************************\n");
  printw("                          TOWR user interface (v1.2.2) \n");
  printw("                            \u00a9 Alexander W. Winkler \n");
  printw("                        https://github.com/ethz-adrl/towr\n");
  printw(" ******************************************************************************\n\n");

  ::ros::NodeHandle n;

  user_command_pub_ = n.advertise<towr_ros::TowrCommand>(towr_msgs::user_command, 1);

  // publish goal zero initially
  goal_geom_.lin.p_.setZero();
  goal_geom_.lin.p_ << 1.0, 0.0, 0.58;
  goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''

  terrain_id_    = 0;
  gait_combo_id_ = 0;
  total_duration_ = 2.4;
  replay_trajectory_ = false;
  optimize_ = false;
  publish_optimized_trajectory_ = false;

  PrintScreen();
}

void
TowrUserInterface::PrintScreen() const
{
  wmove(stdscr, HEADING, X_KEY);
  printw("Key");
  wmove(stdscr, HEADING, X_DESCRIPTION);
  printw("Description");
  wmove(stdscr, HEADING, X_VALUE);
  printw("Value");

  wmove(stdscr, OPTIMIZE, X_KEY);
  printw("o");
  wmove(stdscr, OPTIMIZE, X_DESCRIPTION);
  printw("Optimize motion");
  wmove(stdscr, OPTIMIZE, X_VALUE);
  printw("-");

  wmove(stdscr, REPLAY, X_KEY);
  printw("r");
  wmove(stdscr, REPLAY, X_DESCRIPTION);
  printw("Replay motion");
  wmove(stdscr, REPLAY, X_VALUE);
  printw("-");

  wmove(stdscr, GOAL_POS, X_KEY);
  printw("arrows");
  wmove(stdscr, GOAL_POS, X_DESCRIPTION);
  printw("Goal x-y-z");
  wmove(stdscr, GOAL_POS, X_VALUE);
  PrintVector(goal_geom_.lin.p_);
  printw(" [m]");

  wmove(stdscr, GOAL_ORI, X_KEY);
  printw("keypad");
  wmove(stdscr, GOAL_ORI, X_DESCRIPTION);
  printw("Goal r-p-y");
  wmove(stdscr, GOAL_ORI, X_VALUE);
  PrintVector(goal_geom_.ang.p_);
  printw(" [rad]");

  wmove(stdscr, GAIT, X_KEY);
  printw("g");
  wmove(stdscr, GAIT, X_DESCRIPTION);
  printw("Gait");
  wmove(stdscr, GAIT, X_VALUE);
  printw("%s", std::to_string(gait_combo_id_).c_str());

  wmove(stdscr, TERRAIN, X_KEY);
  printw("t");
  wmove(stdscr, TERRAIN, X_DESCRIPTION);
  printw("Terrain");
  wmove(stdscr, TERRAIN, X_VALUE);
  printw("%i", terrain_id_);

  wmove(stdscr, DURATION, X_KEY);
  printw("+/-");
  wmove(stdscr, DURATION, X_DESCRIPTION);
  printw("Duration");
  wmove(stdscr, DURATION, X_VALUE);
  printw("%f [s]", total_duration_);

  wmove(stdscr, PUBLISH, X_KEY);
  printw("p");
  wmove(stdscr, PUBLISH, X_DESCRIPTION);
  printw("Publish motion-plan");
  wmove(stdscr, PUBLISH, X_VALUE);
  printw("-");

  wmove(stdscr, CLOSE, X_KEY);
  printw("q");
  wmove(stdscr, CLOSE, X_DESCRIPTION);
  printw("Close user interface");
  wmove(stdscr, CLOSE, X_VALUE);
  printw("-");

  wmove(stdscr, Y_STATUS, X_KEY);
  printw("Status:");
}

void
TowrUserInterface::CallbackKey (int c)
{
  const static double d_lin = 0.1;  // [m]
  const static double d_ang = 0.25; // [rad]

  switch (c) {
    case KEY_RIGHT:
      goal_geom_.lin.p_.x() -= d_lin;
      break;
    case KEY_LEFT:
      goal_geom_.lin.p_.x() += d_lin;
      break;
    case KEY_DOWN:
      goal_geom_.lin.p_.y() += d_lin;
      break;
    case KEY_UP:
      goal_geom_.lin.p_.y() -= d_lin;
      break;
    case KEY_PPAGE:
      goal_geom_.lin.p_.z() += 0.5*d_lin;
      break;
    case KEY_NPAGE:
      goal_geom_.lin.p_.z() -= 0.5*d_lin;
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
      break;

    case 'g':
      gait_combo_id_ = AdvanceCircularBuffer(gait_combo_id_, max_gait_id_);
      break;

    // duration
    case '+':
      total_duration_ += 0.2;
    break;
    case '-':
      total_duration_ -= 0.2;
    break;


    case 'o':
      optimize_ = true;
      wmove(stdscr, Y_STATUS, X_DESCRIPTION);
      printw("Optimize motion request sent\n");
      break;
    case 'p':
      publish_optimized_trajectory_ = true;
      wmove(stdscr, Y_STATUS, X_DESCRIPTION);
      printw("Publish optimized trajectory request sent\n");
      break;
    case 'r':
      replay_trajectory_ = true;
      wmove(stdscr, Y_STATUS, X_DESCRIPTION);
      printw("Replaying already optimized trajectory\n");
      break;
    case 'q':
      printw("Closing user interface\n");
      ros::shutdown(); break;
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
  msg.optimize          = optimize_;
  msg.terrain_id        = terrain_id_;
  msg.gait_id           = gait_combo_id_;
  msg.total_duration    = total_duration_;
  msg.publish_traj      = publish_optimized_trajectory_;

  user_command_pub_.publish(msg);

  PrintScreen();

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
  printw("%.3f  %.3f  %.3f", v.x(), v.y(), v.z());
}


} /* namespace towr */


// the actual ros node
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "towr_user_iterface");

  initscr();
  cbreak();              // disables buffering of types characters
  noecho();              // suppresses automatic output of typed characters
  keypad(stdscr, TRUE);  // to capture special keypad characters

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

