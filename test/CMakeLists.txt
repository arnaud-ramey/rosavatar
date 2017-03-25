catkin_add_gtest(gtest_rosavatar       gtest_rosavatar.cpp)
target_link_libraries(gtest_rosavatar  ${catkin_LIBRARIES}
  ${SDL2_LIBRARY} SDL2_gfx SDL2_image SDL2_ttf SDL2_mixer
)
