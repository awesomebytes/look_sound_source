cmake_minimum_required(VERSION 2.8.3)
project(look_sound_source)


find_package(catkin REQUIRED COMPONENTS
  hark_msgs
  rospy
)



catkin_package(
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
 scripts/look_source_audio.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

