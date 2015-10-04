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

\todo Description of the file

\section Parameters
  - \b "~eyes_folder"
    [std::string, default:""]
    Folder where the different images can be found.
    Leave empty for the default face.

\section Subscriptions
  - \b "~iris_position"
    [geometry_msgs/Point]
    Output topic, containing the normalized coordinates of

  - \b "~state"
    [std_msgs/String]
    State. Sad if nobody around, normal otherwise.

\section Publications
  - \b "~eyes"
    [sensors_msgs/Image]
    Generated avatar image.
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// third parties
#include "avatar/eyes_builder.h"

class RosAvatar : public EyeBuilder {
public:
  RosAvatar() : EyeBuilder(), nh_private("~") {
    // get params
    std::string eyes_folder = "";
    nh_private.param("eyes_folder", eyes_folder, eyes_folder);
    if (!load_eyes(eyes_folder)) {
      ROS_FATAL("Could not load eyes folder '%s'\n", eyes_folder.c_str());
      ros::shutdown();
    }
    // create subscribers
    iris_pos_sub = nh_private.subscribe("iris_position", 1, &RosAvatar::iris_pos_cb, this);
    state_sub = nh_private.subscribe("state", 1, &RosAvatar::state_cb, this);
    // create publishers
    img_pub = nh_private.advertise<sensor_msgs::Image>("eyes", 1);
    bridge.encoding = sensor_msgs::image_encodings::BGR8;
    ROS_INFO("rosavatar: getting iris on '%s', state on '%s', publishing image on '%s'",
             iris_pos_sub.getTopic().c_str(), state_sub.getTopic().c_str(),
             img_pub.getTopic().c_str());
  }

  void refresh() {
    if (img_pub.getNumSubscribers() == 0)
      return;
    redraw_eyes();
    get_eyes().copyTo(bridge.image);
    bridge.header.stamp = ros::Time::now();
    img_pub.publish(bridge.toImageMsg());
  }

private:
  void iris_pos_cb(const geometry_msgs::PointConstPtr & msg) {
    // printf("iris_pos_cb()\n");
    move_both_iris(msg->x, msg->y);
  }

  void state_cb(const std_msgs::StringConstPtr & msg) {
    // printf("state_cb()\n");
    set_state(msg->data);
  }

  ros::NodeHandle nh_public, nh_private;
  ros::Subscriber iris_pos_sub, state_sub;
  ros::Publisher img_pub;
  cv_bridge::CvImage bridge;
}; // end class RosAvatar

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosavatar");
  RosAvatar avatar;
  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    avatar.refresh();
    rate.sleep();
  } // end while (ros::ok())
  return 0;
}

