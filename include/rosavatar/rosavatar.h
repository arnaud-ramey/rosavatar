#ifndef ROSAVATAR_H
#define ROSAVATAR_H
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// third parties
#include "avatar/avatar.h"
#include "vision_utils/XmlDocument.h"

std::string replace_find_tags(const std::string & path) {
  std::string out = path, pattern_begin = "$(find ", pattern_end = ")";
  size_t begin = 0, end = 0, pbs = pattern_begin.size();
  while(true) {
    begin = out.find(pattern_begin, begin);
    end   = out.find(pattern_end, begin);
    if (begin == std::string::npos || end == std::string::npos)
      return out;
    std::string pkgname = out.substr(begin+pbs, end-begin-pbs);
    //printf("pkgname:'%s'\n", pkgname.c_str());
    std::string pkgpath = ros::package::getPath(pkgname);
    out.replace(begin, end-begin+1, pkgpath);
    begin += pkgpath.length();
  }
}

class XmlAvatar : public Avatar {
public:
  typedef vision_utils::XmlDocument::Node Node;

  bool from_xml_file(const std::string & filename) {
    clear();
    vision_utils::XmlDocument doc;
    if (!doc.load_from_file(replace_find_tags(filename)))
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
    _bg.setTo(cv::Vec3b(_bg_color[0], _bg_color[1], _bg_color[2]));
    _bg.copyTo(_avatar);

    // get eye nodes
    std::vector<Node*> eye_nodes, center_nodes;
    doc.get_all_nodes_at_direction(doc.root(), "eye", eye_nodes);
    for (unsigned int i = 0; i < eye_nodes.size(); ++i) {
      Node* eye_node = eye_nodes[i];
      std::string folder = doc.get_node_attribute(eye_node, "folder");
      Eye eye;
      DEBUG_PRINT("Eye folder:'%s'\n", folder.c_str());
      // load eye imgs if needed
      if (folder.empty() && !eye.load_default_imgs())
        return false;
      else if (!eye.load_imgs(replace_find_tags(folder)))
        return false;
      std::vector<cv::Point> eye_centers;
      std::vector<bool> eye_flips;
      doc.get_all_nodes_at_direction(eye_node, "center", center_nodes);
      for (unsigned int j = 0; j < center_nodes.size(); ++j) {
        Node* center_node = center_nodes[j];
        int x = doc.get_node_attribute(center_node, "x", 0);
        int y = doc.get_node_attribute(center_node, "y", 0);
        bool flip = doc.get_node_attribute(center_node, "flip", false);
        eye_centers.push_back(cv::Point(x, y));
        eye_flips.push_back(flip);
      } // end for j
      if (!add_eye(eye, eye_centers, eye_flips))
        return false;
    } // end for i

    // get led nodes
    std::vector<Node*> led_nodes;
    doc.get_all_nodes_at_direction(doc.root(), "led", led_nodes);
    for (unsigned int i = 0; i < led_nodes.size(); ++i) {
      Node* led_node = led_nodes[i];
      std::string folder = doc.get_node_attribute(led_node, "folder");
      int center_pos_x = doc.get_node_attribute(led_node, "center_pos_x", 0);
      int center_pos_y = doc.get_node_attribute(led_node, "center_pos_y", 0);
      bool has_auto_mode = doc.has_node_attribute(led_node, "auto_mode_threshold");
      double auto_mode_threshold = doc.get_node_attribute(led_node, "auto_mode_threshold", -1.0);
      Led led;
      DEBUG_PRINT("Led folder:'%s'\n", folder.c_str());
      // load led imgs if needed
      if (!folder.empty() && !led.load_imgs(replace_find_tags(folder)))
        return false;
      if (has_auto_mode)
        led.set_auto_mode(auto_mode_threshold);
      if (!add_led(led, cv::Point(center_pos_x, center_pos_y)))
        return false;
    } // end for i
    return true; // success
  }

}; // end class XmlAvatar

class RosAvatar : public XmlAvatar {
public:
  RosAvatar() : XmlAvatar(), nh_private("~") {
    // get params
    std::string xml_file = "";
    nh_private.param("xml_file", xml_file, xml_file);
    if (xml_file.empty())
      load_default_avatar();
    else
      from_xml_file(xml_file);
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
    for (unsigned int i = 0; i < neyes(); ++i)
      _eyes[i].set_state(msg->data);
  }

  void mouth_vol_cb(const std_msgs::Float64ConstPtr & msg) {
    // printf("mouth_vol_cb()\n");
    for (unsigned int i = 0; i < nleds(); ++i)
      _leds[i].set_auto_mode_value(msg->data);
  }

  ros::NodeHandle nh_public, nh_private;
  ros::Subscriber _iris_pos_sub, _state_sub, _mouth_vol_sub;
  ros::Publisher img_pub;
  cv_bridge::CvImage bridge;
}; // end class RosAvatar

#endif // ROSAVATAR_H
