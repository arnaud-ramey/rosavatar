add_executable(face2point.exe face2point.cpp)
target_link_libraries(face2point.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(rosavatar.exe rosavatar.cpp
 ../include/rosavatar/opencv_face_detector.h)
target_link_libraries(rosavatar.exe ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(win2point.exe win2point.cpp)
target_link_libraries(win2point.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

