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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// third parties
#include "avatar/avatar.h"

class RosAvatar : public Avatar {
public:
  RosAvatar() : Avatar(), nh_private("~") {
    // get params
    int nleds = 5, bg_r = 0, bg_g = 0, bg_b = 0;
    std::string eyes_folder = "", led_prefix = "";
    nh_private.param("led_prefix", led_prefix, led_prefix);
    nh_private.param("eyes_folder", eyes_folder, eyes_folder);
    nh_private.param("nleds", nleds, nleds);
    nh_private.param("bg_r", bg_r, bg_r);
    nh_private.param("bg_g", bg_g, bg_g);
    nh_private.param("bg_b", bg_b, bg_b);
    if (!load_default_avatar(eyes_folder, led_prefix, nleds,
                             bg_r, bg_g, bg_b)) {
      ROS_FATAL("Could not load avatar with eyes '%s' and LEDs '%s'\n",
                eyes_folder.c_str(), led_prefix.c_str());
      ros::shutdown();
    }
    // set leds to turn on according to volume and position
    unsigned int ledw = nleds / 2;
    for (int i = 0; i < nleds; ++i) {
      double thres = 1. * abs(i - ledw) / (ledw+1);
      ROS_INFO("Led %i: setting auto mode at %g.", i, thres);
      _leds[i].set_auto_mode(thres);
    } // end for i
    // create subscribers
    _iris_pos_sub = nh_private.subscribe("iris_position", 1, &RosAvatar::iris_pos_cb, this);
    _mouth_vol_sub = nh_private.subscribe("mouth_vol", 1, &RosAvatar::mouth_vol_cb, this);
    _state_sub = nh_private.subscribe("state", 1, &RosAvatar::state_cb, this);
    // create publishers
    img_pub = nh_private.advertise<sensor_msgs::Image>("avatar_image", 1);
    bridge.encoding = sensor_msgs::image_encodings::BGR8;
    ROS_INFO("rosavatar: getting iris on '%s', state on '%s', publishing image on '%s'",
             _iris_pos_sub.getTopic().c_str(), _state_sub.getTopic().c_str(),
             img_pub.getTopic().c_str());
  }

  bool refresh() {
    if (img_pub.getNumSubscribers() == 0)
      return true;
    if (!redraw())
      return false;
    get_avatar().copyTo(bridge.image);
    bridge.header.stamp = ros::Time::now();
    img_pub.publish(bridge.toImageMsg());
    return true;
  }

private:
  void iris_pos_cb(const geometry_msgs::PointConstPtr & msg) {
    // printf("iris_pos_cb()\n");
    move_iris(msg->x, msg->y);
  }

  void state_cb(const std_msgs::StringConstPtr & msg) {
    // printf("state_cb()\n");
    _eye.set_state(msg->data);
  }

  void mouth_vol_cb(const std_msgs::Float64ConstPtr & msg) {
    // printf("mouth_vol_cb()\n");
    unsigned int nleds = _leds.size();
    for (unsigned int i = 0; i < nleds; ++i)
      _leds[i].set_auto_mode_value(msg->data);
  }

  ros::NodeHandle nh_public, nh_private;
  ros::Subscriber _iris_pos_sub, _state_sub, _mouth_vol_sub;
  ros::Publisher img_pub;
  cv_bridge::CvImage bridge;
}; // end class RosAvatar

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosavatar");
  RosAvatar avatar;
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    avatar.refresh();
    rate.sleep();
  } // end while (ros::ok())
  return 0;
}

