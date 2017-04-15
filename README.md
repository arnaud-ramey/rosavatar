# rosavatar

[![Build Status](https://travis-ci.org/arnaud-ramey/rosavatar.svg)](https://travis-ci.org/arnaud-ramey/rosavatar)

![sample_boy](doc/sample_boy_thumb.png)
![sample_mip](doc/sample_mip_thumb.png)
![sample_octopus](doc/sample_octopus_thumb.png)

This package renders virtual faces that can be controlled with ROS topics.

Parameters
==========

 * ```~eyes_folder```
  [std::string, default:""]
  Folder where the different images can be found.
  Leave empty for the default face.

Subscriptions
=============

 * ```~iris_position```
  [geometry_msgs/Point]
  Output topic, containing the normalized coordinates of

 * ```~mouth_vol```
  [std_msgs/Float64]
  Mouth volume, that will turn LEDS on/off.
  Will be clamped between 0 and 1.

 * ```~state```
  [std_msgs/String]
  State. Sad if nobody around, normal otherwise.

Publications
============

 * ```~eyes```
  [sensors_msgs/Image]
  Generated avatar image.

Example launch files
====================

Mouse control
-------------

This creates a small window in which the user can orientate the look of the robot.
It makes the default avatar look in the direction of the mouse.

`$ roslaunch rosavatar face_control.launch`

To test the volume meter made by the mouth leds, you can play a song, or run:

`$ speaker-test -t wav`

Face control
------------

This makes the default avatar look in the direction of any detected face.
Otherwise, the avatar turns sleepy.

`$ roslaunch rosavatar face_control.launch`

To test the volume meter made by the mouth leds, you can play a song, or run:

`$ speaker-test -t wav`


Depends on [cv_camera](http://wiki.ros.org/cv_camera).
To install:

```bash
$ git clone https://github.com/OTL/cv_camera
$ catkin_make --only-pkg-with-deps rosavatar cv_camera
```

Licence
=======

LGPL v3 (GNU Lesser General Public License version 3).
See LICENCE.

