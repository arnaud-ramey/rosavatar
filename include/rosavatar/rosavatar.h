#ifndef ROSAVATAR_H
#define ROSAVATAR_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
// third parties
#include "rosavatar/sdl_avatar.h"

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
    _state_sub = nh_private.subscribe("eyes_state", 1, &RosAvatar::eyes_state_cb, this);
    // create a subscriber for each renderable
    unsigned int nr = nrenderables();
    for (unsigned int i = 0; i < nr; ++i) {
      switch (_renderables[i]->get_type()) {
        case AvatarRenderable::TYPE_EYE:
          break;
        case AvatarRenderable::TYPE_BINARY_LED:
          _binary_led_state_subs.push_back(nh_public.subscribe<std_msgs::Bool>
                                           (_renderables[i]->get_name() + "/state", 1,
                                            boost::bind(&RosAvatar::binary_led_state_cb, this, _1, i)));
          break;
        case AvatarRenderable::TYPE_COLOR_LED:
          _color_led_color_subs.push_back(nh_public.subscribe<std_msgs::ColorRGBA>
                                           (_renderables[i]->get_name() + "/color", 1,
                                            boost::bind(&RosAvatar::color_led_color_cb, this, _1, i)));
          break;
        default:
          break;
      }
    } // end for i

    ROS_INFO("rosavatar: getting iris on '%s', state on '%s'",
             _iris_pos_sub.getTopic().c_str(), _state_sub.getTopic().c_str());
    return true;
  }

private:
  void iris_pos_cb(const geometry_msgs::PointConstPtr & msg) {
    // printf("iris_pos_cb()\n");
    move_iris(msg->x, msg->y);
  }

  void eyes_state_cb(const std_msgs::StringConstPtr & msg) {
    set_eyes_state(msg->data);
  }

  void mouth_vol_cb(const std_msgs::Float64ConstPtr & msg) {
    // printf("mouth_vol_cb()\n");
    for (unsigned int i = 0; i < nrenderables(); ++i) {
      if (get_rtype(i) == AvatarRenderable::TYPE_BINARY_LED) {
        BinaryLed* renderable = (BinaryLed*) _renderables[i];
        renderable->set_auto_mode_value(msg->data);
      }
    }
  }

  void binary_led_state_cb(const std_msgs::BoolConstPtr & msg, unsigned int i) {
    if (get_rtype(i) != AvatarRenderable::TYPE_BINARY_LED)
      ROS_WARN("Renderable %i is of type %i -> not a binary led", i, get_rtype(i));
    else
      ((BinaryLed*) _renderables[i])->set_state(msg->data ? BinaryLed::ON : BinaryLed::OFF);
  }

  void color_led_color_cb(const std_msgs::ColorRGBAConstPtr & msg, unsigned int i) {
    if (get_rtype(i) != AvatarRenderable::TYPE_COLOR_LED) {
      ROS_WARN("Renderable %i is of type %i -> not a color led", i, get_rtype(i));
      return;
    }
    SDL_Color c = SDL_Color_ctor(255.*msg->r, 255.*msg->g, 255.*msg->b, 255.*msg->a);
    ((ColorLed*) _renderables[i])->set_color(_renderer, c);
  }

  std::vector<ros::Subscriber> _binary_led_state_subs, _color_led_color_subs;
  ros::NodeHandle nh_public, nh_private;
  ros::Subscriber _iris_pos_sub, _state_sub, _mouth_vol_sub;
}; // end class RosAvatar

#endif // ROSAVATAR_H
