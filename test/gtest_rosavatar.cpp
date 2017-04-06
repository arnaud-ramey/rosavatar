/*!
 * \file gtest_gesture_io.cpp
 *
 * Testing gesture_io interface from gesture player package
 *
 * \date Oct 2014
 * \author Irene Pérez
 */

#include <gtest/gtest.h>
#include <rosavatar/rosavatar.h>

std::string datafolder() { return ros::package::getPath("rosavatar") + "/data/avatars/"; }
unsigned int win_flags = 0;//SDL_WINDOWEVENT_HIDDEN;

void test_tags(const std::string & in, const std::string & exp) {
  std::string out = vision_utils::replace_find_tags(in);
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
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.init(win_flags));
  ASSERT_TRUE(avatar.get_bg_color() == SDL_Color_ctor(0,0,0));
  ASSERT_FALSE(avatar.from_xml_file("/error.xml", win_flags));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_empty) {
  SDLAvatar avatar;
  ASSERT_FALSE(avatar.from_xml_file(datafolder() + "avatar_empty.xml", win_flags));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_wh) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_empty_wh.xml", win_flags));
  ASSERT_TRUE(avatar.get_bg_color() == SDL_Color_ctor(10,20,30));
  ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1eye_no_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1eye_no_folder.xml", win_flags));
  ASSERT_TRUE(avatar.neyes() == 1) << "neyes:" << avatar.neyes();
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_1eye_bad_folder) {
  SDLAvatar avatar;
  ASSERT_FALSE(avatar.from_xml_file(datafolder() + "avatar_1eye_bad_folder.xml", win_flags));
}
TEST(TestSuite, avatar_1eye_good_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1eye_good_folder.xml", win_flags));
  ASSERT_TRUE(avatar.neyes() == 1) << "neyes:" << avatar.neyes();
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_2eyes) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_2eyes.xml", win_flags));
  ASSERT_TRUE(avatar.neyes()     == 1) << "neyes:" << avatar.neyes();
  ASSERT_TRUE(avatar.neye_rois() == 2) << "neye_rois:" << avatar.neye_rois();
  ASSERT_TRUE(avatar.render());
  ASSERT_TRUE(avatar.set_eyes_state("laughing"));
  for (unsigned int i = 0; i < 10; ++i)
    ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1led_no_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1led_no_folder.xml", win_flags));
  ASSERT_TRUE(avatar.nbinary_leds() == 1) << "nleds:" << avatar.nbinary_leds();
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_1led_bad_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1led_bad_folder.xml", win_flags));
  ASSERT_TRUE(avatar.nbinary_leds() == 1) << "nleds:" << avatar.nbinary_leds();
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_1led_good_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1led_good_folder.xml", win_flags));
  ASSERT_TRUE(avatar.nbinary_leds() == 1) << "nleds:" << avatar.nbinary_leds();
  ASSERT_TRUE(avatar.get_binary_led(0).get_name() == "led1") << "name:" << avatar.get_binary_led(0).get_name();
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_2leds) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_2leds.xml", win_flags));
  ASSERT_TRUE(avatar.nbinary_leds() == 2) << "nleds:" << avatar.nbinary_leds();
  ASSERT_TRUE(avatar.get_binary_led(0).get_name() == "myled") << "name:" << avatar.get_binary_led(0).get_name();
  ASSERT_TRUE(avatar.get_binary_led(1).get_name() == "led2") << "name:" << avatar.get_binary_led(1).get_name();
  ASSERT_TRUE(avatar.get_binary_led(0).get_auto_mode() == true);
  ASSERT_TRUE(avatar.get_binary_led(0).get_auto_mode_threshold() == 0.5)
      << "thres:" << avatar.get_binary_led(0).get_auto_mode_threshold();
  ASSERT_TRUE(avatar.get_binary_led(1).get_auto_mode() == false);
  ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_2eyes2leds) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_2eyes2leds.xml", win_flags));
  ASSERT_TRUE(avatar.neyes()     == 1) << "neyes:" << avatar.neyes();
  ASSERT_TRUE(avatar.neye_rois() == 2) << "neye_rois:" << avatar.neye_rois();
  ASSERT_TRUE(avatar.nbinary_leds() == 2) << "nleds:" << avatar.nbinary_leds();
  ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_change_state) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1eye_good_folder.xml", win_flags));
  for (unsigned int i = 0; i < 10; ++i)
    ASSERT_TRUE(avatar.render());
  ASSERT_TRUE(avatar.set_eyes_state("angry"));
  for (unsigned int i = 0; i < 10; ++i)
    ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_mini) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_mini.xml", win_flags));
  ASSERT_TRUE(avatar.neyes()     == 1) << "neyes:" << avatar.neyes();
  ASSERT_TRUE(avatar.neye_rois() == 2) << "neye_rois:" << avatar.neye_rois();
  ASSERT_TRUE(avatar.nbinary_leds() == 6) << "nleds:" << avatar.nbinary_leds();
  ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_octopus) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_octopus.xml", win_flags));
  for (unsigned int i = 0; i < 10; ++i)
    ASSERT_TRUE(avatar.render());
  ASSERT_TRUE(avatar.set_eyes_state("laughing"));
  for (unsigned int i = 0; i < 10; ++i)
    ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST() or TEST_F()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

