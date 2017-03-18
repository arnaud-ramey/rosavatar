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

TEST(TestSuite, load_non_existing) {
  XmlAvatar avatar;
  ASSERT_TRUE(avatar.get_bg_color() == cv::Scalar(0,0,0));
  ASSERT_FALSE(avatar.from_file("/error.xml"));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_empty) {
  XmlAvatar avatar;
  ASSERT_FALSE(avatar.from_file(datafolder() + "avatar_empty.xml"));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_wh) {
  XmlAvatar avatar;
  ASSERT_TRUE(avatar.from_file(datafolder() + "avatar_empty_wh.xml"));
  ASSERT_TRUE(avatar.get_bg_color() == cv::Scalar(30,20,10));
  ASSERT_TRUE(avatar.get_avatar_size() == cv::Size(1600,1200));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1eye_no_folder) {
  XmlAvatar avatar;
  ASSERT_TRUE(avatar.from_file(datafolder() + "avatar_1eye_no_folder.xml"));
  ASSERT_TRUE(avatar.neyes() == 1);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1eye_bad_folder) {
  XmlAvatar avatar;
  ASSERT_TRUE(avatar.from_file(datafolder() + "avatar_1eye_bad_folder.xml"));
  ASSERT_TRUE(avatar.neyes() == 1);
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv){
  // Run all the tests that were declared with TEST() or TEST_F()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

