#ifndef ROSAVATAR_H
#define ROSAVATAR_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// third parties
#include "avatar/avatar.h"
#include "vision_utils/XmlDocument.h"

class XmlAvatar : public Avatar {
public:
  typedef vision_utils::XmlDocument::Node Node;

  bool from_file(const std::string & filename) {
    clear();
    vision_utils::XmlDocument doc;
    if (!doc.load_from_file(filename))
      return false;
    // get background property
    Node* bg = doc.get_node_at_direction(doc.root(), "background");
    if (!bg) {
      printf("Missing background node!\n");
      return false;
    }
    int r = doc.get_node_attribute(bg, "r", 0);
    int g = doc.get_node_attribute(bg, "g", 0);
    int b = doc.get_node_attribute(bg, "b", 0);
    int w = doc.get_node_attribute(bg, "width", 640);
    int h = doc.get_node_attribute(bg, "height", 480);
    _bg_color = CV_RGB(r, g, b);
    _bg.create(h, w);
    _bg.setTo(cv::Vec3b(_bg_color[2], _bg_color[1], _bg_color[0]));
    _bg.copyTo(_avatar);
    // get eye nodes
    std::vector<Node*> eye_nodes;
    doc.get_all_nodes_at_direction(doc.root(), "eye", eye_nodes);
    for (unsigned int i = 0; i < eye_nodes.size(); ++i) {
      Node* eye_node = eye_nodes[i];
      std::string folder = doc.get_node_attribute(eye_node, "folder", "");
      int center_pos_x = doc.get_node_attribute(eye_node, "center_pos_x", 0);
      int center_pos_y = doc.get_node_attribute(eye_node, "center_pos_y", 0);
      bool flip = doc.get_node_attribute(eye_node, "flip", false);
      Eye eye;
      if (!folder.empty() && !eye.load_imgs(folder)) // load eye imgs if needed
        return false;
      if (!add_eye(eye, cv::Point(center_pos_x, center_pos_y), flip))
        return false;
    } // end for i
    return true; // success
  }

}; // end class XmlAvatar

class RosAvatar : public XmlAvatar {
public:
  RosAvatar() : XmlAvatar(), nh_private("~") {
    // get params
    //    int nleds = 5, bg_r = 0, bg_g = 0, bg_b = 0;
    //    std::string eyes_folder = "", led_prefix = "";
    //    nh_private.param("led_prefix", led_prefix, led_prefix);
    //    nh_private.param("eyes_folder", eyes_folder, eyes_folder);
    //    nh_private.param("nleds", nleds, nleds);
    //    nh_private.param("bg_r", bg_r, bg_r);
    //    nh_private.param("bg_g", bg_g, bg_g);
    //    nh_private.param("bg_b", bg_b, bg_b);
    //    if (!load_default_avatar(eyes_folder, led_prefix, nleds,
    //                             bg_r, bg_g, bg_b)) {
    //      ROS_FATAL("Could not load avatar with eyes '%s' and LEDs '%s'\n",
    //                eyes_folder.c_str(), led_prefix.c_str());
    //      ros::shutdown();
    //    }
    //    // set leds to turn on according to volume and position
    //    unsigned int ledw = nleds / 2;
    //    for (int i = 0; i < nleds; ++i) {
    //      double thres = 1. * abs(i - ledw) / (ledw+1);
    //      ROS_INFO("Led %i: setting auto mode at %g.", i, thres);
    //      _leds[i].set_auto_mode(thres);
    //    } // end for i
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

#endif // ROSAVATAR_H
