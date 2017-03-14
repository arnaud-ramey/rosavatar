/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/10/4

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A GUI for moving eyes.
\section Publications
  - \b "~out"
    [geometry_msgs/Point]
    Output topic, containing the normalized coordinates of
    the mouse in the window
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

ros::Publisher point_pub, state_pub;
std::string win_name = "win2point";
int h = 100;
cv::Mat3b gui(h, h);
geometry_msgs::Point pt;
std::vector<std::string> states;
unsigned int state_idx = 0, nstates;

void redraw() {
  gui.setTo(cv::Vec3b(255, 255, 255));
  cv::circle(gui, cv::Point(h/2,h/2), h/2, CV_RGB(0,0,0), 1);
  cv::putText(gui, "Mouse here", cv::Point(4, h/2), cv::FONT_HERSHEY_PLAIN,
              1, CV_RGB(0,0,0));
  cv::putText(gui, states[state_idx], cv::Point(4, 2 * h / 3), cv::FONT_HERSHEY_PLAIN,
              1, CV_RGB(0,0,0));
  cv::imshow(win_name, gui);
  // publish new state
  std_msgs::String state_msg;
  state_msg.data = states[state_idx];
  state_pub.publish(state_msg);
}

void mouse_cb(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
  //printf("mouse_cb:x:%i, y:%i\n", x, y);
  pt.x = 2. * x / h - 1;
  pt.y = 2. * y / h - 1;
  point_pub.publish(pt);
  if (event == CV_EVENT_LBUTTONDOWN) {
    state_idx = (state_idx+1) % nstates;
    redraw();
  }
  else if (event == CV_EVENT_RBUTTONDOWN) {
    state_idx = (state_idx+nstates-1) % nstates;
    redraw();
  }
} // end mouse_cb();

int main(int argc, char** argv) {
  ros::init(argc, argv, win_name);
  ros::NodeHandle nh_private("~");
  point_pub = nh_private.advertise<geometry_msgs::Point>("point", 1);
  state_pub = nh_private.advertise<std_msgs::String>("state", 1);
  states.push_back("angry");
  states.push_back("laughing");
  states.push_back("normal");
  states.push_back("sad");
  states.push_back("sleeping");
  states.push_back("sleepy");
  states.push_back("surprised");
  nstates = states.size();
  printf("win2point: emitting Point to '%s', state to '%s'\n",
         point_pub.getTopic().c_str(), state_pub.getTopic().c_str());
  cv::namedWindow(win_name);
  cv::setMouseCallback(win_name, mouse_cb, NULL);
  redraw();

  while (ros::ok()) {
    ros::spinOnce();
    cv::waitKey(50);
  } // end while (ros::ok())
}

