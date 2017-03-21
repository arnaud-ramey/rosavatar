/*!
 * \file gtest_gesture_io.cpp
 *
 * Testing gesture_io interface from gesture player package
 *
 * \date Oct 2014
 * \author Irene Pérez
 */

#include <gtest/gtest.h>
#include <ros/package.h>
#include <rosavatar/rosavatar.h>

std::string datafolder() { return ros::package::getPath("rosavatar") + "/data/"; }

void test_tags(const std::string & in, const std::string & exp) {
  std::string out = replace_find_tags(in);
  ASSERT_TRUE(out == exp) << "out:" << out << ", exp:" << exp;
}

TEST(TestSuite, roscpp) { test_tags("$(find roscpp)", ros::package::getPath("roscpp"));}
TEST(TestSuite, roslib) { test_tags("$(find roslib)", ros::package::getPath("roslib"));}
TEST(TestSuite, rosavatar) { test_tags("$(find rosavatar)", ros::package::getPath("rosavatar"));}
TEST(TestSuite, rosavatar2) {
  test_tags("foo$(find rosavatar)bar",
            std::string("foo")+ros::package::getPath("rosavatar")+"bar");
}

////////////////////////////////////////////////////////////////////////////////


TEST(TestSuite, load_non_existing) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.get_bg_color() == cv::Scalar(0,0,0));
  ASSERT_FALSE(xml_avatar.from_xml_file("/error.xml"));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_empty) {
  XmlAvatar xml_avatar;
  ASSERT_FALSE(xml_avatar.from_xml_file(datafolder() + "avatar_empty.xml"));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_wh) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_empty_wh.xml"));
  ASSERT_TRUE(xml_avatar.get_bg_color() == cv::Scalar(30,20,10));
  ASSERT_TRUE(xml_avatar.get_avatar_size() == cv::Size(1600,1200));
  ASSERT_TRUE(xml_avatar.redraw());
  cv::Mat3b avatar;
  xml_avatar.get_avatar().copyTo(avatar);
  ASSERT_TRUE(avatar.size() == xml_avatar.get_avatar_size());
  ASSERT_TRUE(avatar.at<cv::Vec3b>(0,0) ==  cv::Vec3b(30,20,10))
      << avatar.at<cv::Vec3b>(0,0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1eye_no_folder) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_1eye_no_folder.xml"));
  ASSERT_TRUE(xml_avatar.neyes() == 1) << "neyes:" << xml_avatar.neyes();
  ASSERT_TRUE(xml_avatar.redraw());
}
TEST(TestSuite, avatar_1eye_bad_folder) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_1eye_bad_folder.xml"));
  ASSERT_TRUE(xml_avatar.neyes() == 1) << "neyes:" << xml_avatar.neyes();
  ASSERT_TRUE(xml_avatar.redraw());
}
TEST(TestSuite, avatar_1eye_good_folder) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_1eye_good_folder.xml"));
  ASSERT_TRUE(xml_avatar.neyes() == 1) << "neyes:" << xml_avatar.neyes();
  ASSERT_TRUE(xml_avatar.redraw());
}
TEST(TestSuite, avatar_2eyes) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_2eyes.xml"));
  ASSERT_TRUE(xml_avatar.neyes()     == 1) << "neyes:" << xml_avatar.neyes();
  ASSERT_TRUE(xml_avatar.neye_rois() == 2) << "neye_rois:" << xml_avatar.neye_rois();
  ASSERT_TRUE(xml_avatar.redraw());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1led_no_folder) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_1led_no_folder.xml"));
  ASSERT_TRUE(xml_avatar.nleds() == 1) << "nleds:" << xml_avatar.nleds();
  ASSERT_TRUE(xml_avatar.redraw());
}
TEST(TestSuite, avatar_1led_bad_folder) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_1led_bad_folder.xml"));
  ASSERT_TRUE(xml_avatar.nleds() == 1) << "nleds:" << xml_avatar.nleds();
  ASSERT_TRUE(xml_avatar.redraw());
}
TEST(TestSuite, avatar_1led_good_folder) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_1led_good_folder.xml"));
  ASSERT_TRUE(xml_avatar.nleds() == 1) << "nleds:" << xml_avatar.nleds();
  ASSERT_TRUE(xml_avatar.redraw());
}
TEST(TestSuite, avatar_2leds) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_2leds.xml"));
  ASSERT_TRUE(xml_avatar.nleds() == 2) << "nleds:" << xml_avatar.nleds();
  ASSERT_TRUE(xml_avatar.get_led(0).get_auto_mode() == true);
  ASSERT_TRUE(xml_avatar.get_led(0).get_auto_mode_threshold() == 0.5)
      << "thres:" << xml_avatar.get_led(0).get_auto_mode_threshold();
  ASSERT_TRUE(xml_avatar.get_led(1).get_auto_mode() == false);
  ASSERT_TRUE(xml_avatar.redraw());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_2eyes2leds) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_2eyes2leds.xml"));
  ASSERT_TRUE(xml_avatar.neyes()     == 1) << "neyes:" << xml_avatar.neyes();
  ASSERT_TRUE(xml_avatar.neye_rois() == 2) << "neye_rois:" << xml_avatar.neye_rois();
  ASSERT_TRUE(xml_avatar.nleds() == 2) << "nleds:" << xml_avatar.nleds();
  ASSERT_TRUE(xml_avatar.redraw());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_mini) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_mini.xml"));
  ASSERT_TRUE(xml_avatar.neyes()     == 1) << "neyes:" << xml_avatar.neyes();
  ASSERT_TRUE(xml_avatar.neye_rois() == 2) << "neye_rois:" << xml_avatar.neye_rois();
  ASSERT_TRUE(xml_avatar.nleds() == 6) << "nleds:" << xml_avatar.nleds();
  ASSERT_TRUE(xml_avatar.redraw());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_octopus) {
  XmlAvatar xml_avatar;
  ASSERT_TRUE(xml_avatar.from_xml_file(datafolder() + "avatar_octopus.xml"));
  ASSERT_TRUE(xml_avatar.redraw());
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST() or TEST_F()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

