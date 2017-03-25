add_executable(face2point.exe face2point.cpp)
target_link_libraries(face2point.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(rosavatar.exe rosavatar.cpp
  ../include/rosavatar/sdl_avatar.h
  ../include/rosavatar/rosavatar.h)
target_link_libraries(rosavatar.exe ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${SDL2_LIBRARY} SDL2_gfx SDL2_image SDL2_ttf SDL2_mixer)

add_executable(win2point.exe win2point.cpp)
target_link_libraries(win2point.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

