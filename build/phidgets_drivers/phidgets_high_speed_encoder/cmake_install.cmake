# Install script for directory: /home/alfie/ROCO_318/ROCO318_Phidget1044_CW/src/phidgets_drivers/phidgets_high_speed_encoder

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phidgets_high_speed_encoder/msg" TYPE FILE FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/src/phidgets_drivers/phidgets_high_speed_encoder/msg/EncoderDecimatedSpeed.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phidgets_high_speed_encoder/cmake" TYPE FILE FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/build/phidgets_drivers/phidgets_high_speed_encoder/catkin_generated/installspace/phidgets_high_speed_encoder-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/include/phidgets_high_speed_encoder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/share/roseus/ros/phidgets_high_speed_encoder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/share/common-lisp/ros/phidgets_high_speed_encoder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/share/gennodejs/ros/phidgets_high_speed_encoder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/lib/python2.7/dist-packages/phidgets_high_speed_encoder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/lib/python2.7/dist-packages/phidgets_high_speed_encoder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/build/phidgets_drivers/phidgets_high_speed_encoder/catkin_generated/installspace/phidgets_high_speed_encoder.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phidgets_high_speed_encoder/cmake" TYPE FILE FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/build/phidgets_drivers/phidgets_high_speed_encoder/catkin_generated/installspace/phidgets_high_speed_encoder-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phidgets_high_speed_encoder/cmake" TYPE FILE FILES
    "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/build/phidgets_drivers/phidgets_high_speed_encoder/catkin_generated/installspace/phidgets_high_speed_encoderConfig.cmake"
    "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/build/phidgets_drivers/phidgets_high_speed_encoder/catkin_generated/installspace/phidgets_high_speed_encoderConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phidgets_high_speed_encoder" TYPE FILE FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/src/phidgets_drivers/phidgets_high_speed_encoder/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder" TYPE EXECUTABLE FILES "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder"
         OLD_RPATH "/home/alfie/ROCO_318/ROCO318_Phidget1044_CW/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/phidgets_high_speed_encoder/phidgets_high_speed_encoder")
    endif()
  endif()
endif()

