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
#include <towr/terrain/height_map.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/robot_model.h>


namespace towr {


enum YCursorRows {HEADING=6, OPTIMIZE=8, VISUALIZE, INITIALIZATION, PLOT,
                  REPLAY_SPEED, GOAL_POS, GOAL_ORI, ROBOT,
                  GAIT, OPTIMIZE_GAIT, TERRAIN, DURATION, CLOSE, END};
static constexpr int Y_STATUS      = END+1;
static constexpr int X_KEY         = 1;
static constexpr int X_DESCRIPTION = 10;
static constexpr int X_VALUE       = 35;


TowrUserInterface::TowrUserInterface ()
{
  printw(" ******************************************************************************\n");
  printw("                          TOWR user interface (v1.4) \n");
  printw("                            \u00a9 Alexander W. Winkler \n");
  printw("                        https://github.com/ethz-adrl/towr\n");
  printw(" ******************************************************************************\n\n");

  ::ros::NodeHandle n;
  user_command_pub_ = n.advertise<towr_ros::TowrCommand>(towr_msgs::user_command, 1);

  goal_geom_.lin.p_.setZero();
  goal_geom_.lin.p_ << 2.1, 0.0, 0.0;
  goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''

  robot_      = RobotModel::Monoped;
  terrain_    = HeightMap::FlatID;
  gait_combo_ = GaitGenerator::C0;
  total_duration_ = 2.4;
  visualize_trajectory_ = false;
  plot_trajectory_ = false;
  replay_speed_ = 1.0; // realtime
  optimize_ = false;
  publish_optimized_trajectory_ = false;
  optimize_phase_durations_ = false;

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
  printw("Info");

  wmove(stdscr, OPTIMIZE, X_KEY);
  printw("o");
  wmove(stdscr, OPTIMIZE, X_DESCRIPTION);
  printw("Optimize motion");
  wmove(stdscr, OPTIMIZE, X_VALUE);
  printw("-");

  wmove(stdscr, VISUALIZE, X_KEY);
  printw("v");
  wmove(stdscr, VISUALIZE, X_DESCRIPTION);
  printw("visualize motion in rviz");
  wmove(stdscr, VISUALIZE, X_VALUE);
  printw("-");

  wmove(stdscr, INITIALIZATION, X_KEY);
  printw("i");
  wmove(stdscr, INITIALIZATION, X_DESCRIPTION);
  printw("play initialization");
  wmove(stdscr, INITIALIZATION, X_VALUE);
  printw("-");

  wmove(stdscr, PLOT, X_KEY);
  printw("p");
  wmove(stdscr, PLOT, X_DESCRIPTION);
  printw("Plot values (rqt_bag)");
  wmove(stdscr, PLOT, X_VALUE);
  printw("-");

  wmove(stdscr, REPLAY_SPEED, X_KEY);
  printw(";/'");
  wmove(stdscr, REPLAY_SPEED, X_DESCRIPTION);
  printw("Replay speed");
  wmove(stdscr, REPLAY_SPEED, X_VALUE);
  printw("%.2f", replay_speed_);

  wmove(stdscr, GOAL_POS, X_KEY);
  printw("arrows");
  wmove(stdscr, GOAL_POS, X_DESCRIPTION);
  printw("Goal x-y");
  wmove(stdscr, GOAL_POS, X_VALUE);
  PrintVector2D(goal_geom_.lin.p_.topRows(2));
  printw(" [m]");

  wmove(stdscr, GOAL_ORI, X_KEY);
  printw("keypad");
  wmove(stdscr, GOAL_ORI, X_DESCRIPTION);
  printw("Goal r-p-y");
  wmove(stdscr, GOAL_ORI, X_VALUE);
  PrintVector(goal_geom_.ang.p_);
  printw(" [rad]");

  wmove(stdscr, ROBOT, X_KEY);
  printw("r");
  wmove(stdscr, ROBOT, X_DESCRIPTION);
  printw("Robot");
  wmove(stdscr, ROBOT, X_VALUE);
  printw("%s\n", robot_names.at(static_cast<RobotModel::Robot>(robot_)).c_str());

  wmove(stdscr, GAIT, X_KEY);
  printw("g");
  wmove(stdscr, GAIT, X_DESCRIPTION);
  printw("Gait");
  wmove(stdscr, GAIT, X_VALUE);
  printw("%s", std::to_string(gait_combo_).c_str());

  wmove(stdscr, OPTIMIZE_GAIT, X_KEY);
  printw("y");
  wmove(stdscr, OPTIMIZE_GAIT, X_DESCRIPTION);
  printw("Optimize gait");
  wmove(stdscr, OPTIMIZE_GAIT, X_VALUE);
  optimize_phase_durations_? printw("On\n") : printw("off\n");

  wmove(stdscr, TERRAIN, X_KEY);
  printw("t");
  wmove(stdscr, TERRAIN, X_DESCRIPTION);
  printw("Terrain");
  wmove(stdscr, TERRAIN, X_VALUE);
  printw("%s\n", terrain_names.at(static_cast<HeightMap::TerrainID>(terrain_)).c_str());

  wmove(stdscr, DURATION, X_KEY);
  printw("+/-");
  wmove(stdscr, DURATION, X_DESCRIPTION);
  printw("Duration");
  wmove(stdscr, DURATION, X_VALUE);
  printw("%.2f [s]", total_duration_);

  wmove(stdscr, CLOSE, X_KEY);
  printw("q");
  wmove(stdscr, CLOSE, X_DESCRIPTION);
  printw("Close user interface");
  wmove(stdscr, CLOSE, X_VALUE);
  printw("-");
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
      terrain_ = AdvanceCircularBuffer(terrain_, HeightMap::TERRAIN_COUNT);
      break;

    case 'g':
      gait_combo_ = AdvanceCircularBuffer(gait_combo_, GaitGenerator::COMBO_COUNT);
      break;

    case 'r':
      robot_ = AdvanceCircularBuffer(robot_, RobotModel::ROBOT_COUNT);
      break;

    // duration
    case '+':
      total_duration_ += 0.2;
    break;
    case '-':
      total_duration_ -= 0.2;
    break;
    case '\'':
      replay_speed_ += 0.1;
    break;
    case ';':
      replay_speed_ -= 0.1;
    break;
    case 'y':
      optimize_phase_durations_ = !optimize_phase_durations_;
      break;


    case 'o':
      optimize_ = true;
      wmove(stdscr, Y_STATUS, 0);
      printw("Optimizing motion\n\n");
      break;
    case 'v':
      visualize_trajectory_ = true;
      wmove(stdscr, Y_STATUS, 0);
      printw("Visualizing current bag file\n\n");
      break;
    case 'i':
      play_initialization_ = true;
      wmove(stdscr, Y_STATUS, 0);
      printw("Visualizing initialization (iteration 0)\n\n");
      break;
    case 'p':
      plot_trajectory_ = true;
      wmove(stdscr, Y_STATUS, 0);
      printw("In rqt_bag: right-click on xpp/state_des -> View -> Plot.\n"
             "Then expand the values you wish to plot on the right\n");
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
  msg.goal_lin                 = xpp::Convert::ToRos(goal_geom_.lin);
  msg.goal_ang                 = xpp::Convert::ToRos(goal_geom_.ang);
  msg.total_duration           = total_duration_;
  msg.replay_trajectory        = visualize_trajectory_;
  msg.play_initialization      = play_initialization_;
  msg.replay_speed             = replay_speed_;
  msg.optimize                 = optimize_;
  msg.terrain                  = terrain_;
  msg.gait                     = gait_combo_;
  msg.robot                    = robot_;
  msg.optimize_phase_durations = optimize_phase_durations_;
  msg.plot_trajectory          = plot_trajectory_;

  user_command_pub_.publish(msg);

  PrintScreen();

  optimize_ = false;
  visualize_trajectory_ = false;
  plot_trajectory_ = false;
  play_initialization_ = false;
  publish_optimized_trajectory_ = false;
}

int TowrUserInterface::AdvanceCircularBuffer(int& curr, int max) const
{
  return curr==(max-1)? 0 : curr+1;
}

void
TowrUserInterface::PrintVector(const Eigen::Vector3d& v) const
{
  printw("%.2f  %.2f  %.2f", v.x(), v.y(), v.z());
}

void
TowrUserInterface::PrintVector2D(const Eigen::Vector2d& v) const
{
  printw("%.2f  %.2f", v.x(), v.y());
}

} /* namespace towr */


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

