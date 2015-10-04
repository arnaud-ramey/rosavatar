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

Transform faces in the camera image into directions for the eyes.

\section Parameters
  - \b "~display"
    [bool, default false]
    True to call the display function upon receiving the images.

\section Subscriptions
  - \b {rgb_topic}
    [sensor_msgs::Image]
    The RGB streams.

\section Publications
  - \b "~out"
    [geometry_msgs/Point]
    Output topic, containing the normalized coordinates of

  - \b "~state"
    [std_msgs/String]
    State. Sad if nobody around, normal otherwise.
 */
#include "rosavatar/opencv_face_detector.h"
// opencv
#include <opencv2/highgui/highgui.hpp>
// ROS
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

class Face2Point {
public:
  static const unsigned int QUEUE_SIZE = 3;

  Face2Point() : _nh_private("~"), _it(_nh_public) {
    _classifier = image_utils::create_face_classifier();
    // params
    _nh_private.param("display", _display, false);
    // publishers and subscribers
    _point_pub = _nh_private.advertise<geometry_msgs::Point>("out", 1);
    _state_pub = _nh_private.advertise<std_msgs::String>("state", 1);
    _rgb_sub = _it.subscribe("rgb", QUEUE_SIZE, &Face2Point::rgb_cb, this);
  }
  ~Face2Point() {}

  //////////////////////////////////////////////////////////////////////////////

  virtual void display(const cv::Mat3b & rgb) {
    rgb.copyTo(_rgb_out);
    for (unsigned int user_idx = 0; user_idx < _faces.size(); ++user_idx)
      cv::rectangle(_rgb_out, _faces[user_idx], CV_RGB(0, 255, 0), 2);
    cv::flip(_rgb_out, _rgb_out, 1); // mirror effect
    cv::imshow("face2point", _rgb_out);
    cv::waitKey(5);
  }

protected:

  /*! where the real image processing work is done. */
  virtual void process_rgb(const cv::Mat3b & rgb) {
    image_utils::detect_with_opencv
        (rgb, _classifier, _small_rgb, _faces);
    geometry_msgs::Point pt;
    std_msgs::String state;
    state.data = "sleepy";
    if (_faces.size() > 0) {
      cv::Rect r = _faces.front();
      pt.x =  1. - 2. * (r.x + r.width/2)  / rgb.cols;
      pt.y = -1. + 2. * (r.y + r.height/2) / rgb.rows;
      state.data = "normal";
    }
    _state_pub.publish(state);
    _point_pub.publish(pt);
  } // end process_rgb();

  //////////////////////////////////////////////////////////////////////////////

  void rgb_cb(const sensor_msgs::Image::ConstPtr& rgb_msg) {
    // DEBUG_PRINT("rgb_cb()\n");
    try {
      _rgb_bridge = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat* rgb_ptr = &(_rgb_bridge->image);
    process_rgb(*rgb_ptr);
    if (_display)
      display(*rgb_ptr);
  } // end rgb_cb();

  //////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public, _nh_private;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _rgb_sub;
  cv_bridge::CvImageConstPtr _rgb_bridge;
  bool _display; //!< true to call display

  // create a classifier
  cv::CascadeClassifier _classifier;
  cv::Mat3b _small_rgb, _rgb_out;
  std::vector<cv::Rect> _faces;
  ros::Publisher _point_pub, _state_pub;
}; // end class Face2Point

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "face2point");
  Face2Point f;
  ros::spin();
}
