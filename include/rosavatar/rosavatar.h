#ifndef ROSAVATAR_H
#define ROSAVATAR_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
// third parties
#include "rosavatar/sdl_avatar.h"
//#include "avatar/avatar.h"

class RosAvatar : public SDLAvatar {
public:
  RosAvatar() : nh_private("~") {}

  bool init(Uint32 win_flags = 0) {
    // get params
    std::string xml_file = "";
    nh_private.param("xml_file", xml_file, xml_file);
    if (xml_file.empty() && !load_default_avatar())
      return false;
    else if (!from_xml_file(xml_file, win_flags))
      return false;
    // create subscribers
    _iris_pos_sub = nh_private.subscribe("iris_position", 1, &RosAvatar::iris_pos_cb, this);
    _mouth_vol_sub = nh_private.subscribe("mouth_vol", 1, &RosAvatar::mouth_vol_cb, this);
    _state_sub = nh_private.subscribe("state", 1, &RosAvatar::state_cb, this);
    ROS_INFO("rosavatar: getting iris on '%s', state on '%s'",
             _iris_pos_sub.getTopic().c_str(), _state_sub.getTopic().c_str());
    return true;
  }

private:
  void iris_pos_cb(const geometry_msgs::PointConstPtr & msg) {
    // printf("iris_pos_cb()\n");
    move_iris(msg->x, msg->y);
  }

  void state_cb(const std_msgs::StringConstPtr & msg) {
    for (unsigned int i = 0; i < neyes(); ++i)
      _eyes[i].set_state(msg->data);
  }

  void mouth_vol_cb(const std_msgs::Float64ConstPtr & msg) {
    // printf("mouth_vol_cb()\n");
    for (unsigned int i = 0; i < nrenderables(); ++i) {
      if (get_rtype(i) == AvatarRenderable::BINARY_LED) {
        BinaryLed* renderable = (BinaryLed*) _renderables[i];
        renderable->set_auto_mode_value(msg->data);
      }
    }
  }

  SDL_Renderer* _renderer;
  ros::NodeHandle nh_public, nh_private;
  ros::Subscriber _iris_pos_sub, _state_sub, _mouth_vol_sub;
}; // end class RosAvatar

#endif // ROSAVATAR_H
