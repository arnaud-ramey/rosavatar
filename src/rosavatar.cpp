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
 */
#include "rosavatar/rosavatar.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosavatar");
  RosAvatar avatar;
  if (!avatar.init()) {
    ROS_FATAL("RosAvatar::init() failed!\n");
    return -1;
  }
  ros::Rate rate(15);
  while (ros::ok()) {
    ros::spinOnce();
    if (!avatar.render()) {
      ROS_FATAL("RosAvatar::refresh() failed!\n");
      return -1;
    }
    rate.sleep();
  } // end while (ros::ok())
  return 0;
}

