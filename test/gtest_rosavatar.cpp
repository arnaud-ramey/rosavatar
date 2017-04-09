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

void assert_renderables(SDLAvatar & avatar, int neyes, int nbinary_leds,
                        int ncolor_leds, int nrenderables_rois) {
  ASSERT_EQ(avatar.neyes(), neyes);
  ASSERT_EQ(avatar.nbinary_leds(), nbinary_leds);
  ASSERT_EQ(avatar.ncolor_leds(), ncolor_leds);
  ASSERT_EQ(avatar.nrenderables(), neyes + nbinary_leds + ncolor_leds);
  ASSERT_EQ(avatar.nrenderables_rois(), nrenderables_rois);
}

////////////////////////////////////////////////////////////////////////////////

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
  assert_renderables(avatar, 0, 0, 0,  0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_empty) {
  SDLAvatar avatar;
  ASSERT_FALSE(avatar.from_xml_file(datafolder() + "avatar_empty.xml", win_flags));
  assert_renderables(avatar, 0, 0, 0,  0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_wh) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_empty_wh.xml", win_flags));
  ASSERT_TRUE(avatar.get_bg_color() == SDL_Color_ctor(10,20,30));
  ASSERT_TRUE(avatar.render());
  assert_renderables(avatar, 0, 0, 0,  0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1eye_no_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1eye_no_folder.xml", win_flags));
  assert_renderables(avatar, 1, 0, 0,  1);
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_1eye_bad_folder) {
  SDLAvatar avatar;
  ASSERT_FALSE(avatar.from_xml_file(datafolder() + "avatar_1eye_bad_folder.xml", win_flags));
  assert_renderables(avatar, 0, 0, 0,  0);
}
TEST(TestSuite, avatar_1eye_good_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1eye_good_folder.xml", win_flags));
  assert_renderables(avatar, 1, 0, 0,  1);
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_2eyes) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_2eyes.xml", win_flags));
  assert_renderables(avatar, 1, 0, 0,  2);
  ASSERT_TRUE(avatar.render());
  ASSERT_TRUE(avatar.set_eyes_state("laughing"));
  for (unsigned int i = 0; i < 10; ++i)
    ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_1led_no_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1led_no_folder.xml", win_flags));
  assert_renderables(avatar, 0, 1, 0,  1);
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_1led_bad_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1led_bad_folder.xml", win_flags));
  assert_renderables(avatar, 0, 1, 0,  1);
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_1led_good_folder) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_1led_good_folder.xml", win_flags));
  assert_renderables(avatar, 0, 1, 0,  1);
  ASSERT_TRUE(avatar.get_renderable(0)->get_name() == "binary_led1")
      << "name:" << avatar.get_renderable(0)->get_name();
  ASSERT_TRUE(avatar.render());
}
TEST(TestSuite, avatar_2leds) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_2leds.xml", win_flags));
  assert_renderables(avatar, 0, 2, 0,  2);
  ASSERT_TRUE(avatar.get_rtype(0) == AvatarRenderable::TYPE_BINARY_LED);
  ASSERT_TRUE(avatar.get_rtype(1) == AvatarRenderable::TYPE_BINARY_LED);
  BinaryLed* l0 = (BinaryLed*) avatar.get_renderable(0);
  BinaryLed* l1 = (BinaryLed*) avatar.get_renderable(1);
  ASSERT_TRUE(l0->get_name() == "myled") << "name:" << l0->get_name();
  ASSERT_TRUE(l1->get_name() == "binary_led2") << "name:" << l1->get_name();
  ASSERT_TRUE(l0->get_auto_mode() == true);
  ASSERT_TRUE(l0->get_auto_mode_threshold() == 0.5)
      << "thres:" << l0->get_auto_mode_threshold();
  ASSERT_TRUE(l1->get_auto_mode() == false);
  ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_2eyes2leds) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_2eyes2leds.xml", win_flags));
  assert_renderables(avatar, 1, 2, 0,  4);
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
  assert_renderables(avatar, 1, 6, 0,  8);
  ASSERT_TRUE(avatar.render());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, avatar_octopus) {
  SDLAvatar avatar;
  ASSERT_TRUE(avatar.from_xml_file(datafolder() + "avatar_octopus.xml", win_flags));
  assert_renderables(avatar, 1, 7, 0,  22);
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

