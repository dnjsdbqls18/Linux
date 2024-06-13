amap@aMAP:~/w_catkin_ws$ cmw
Base path: /home/amap/w_catkin_ws
Source space: /home/amap/w_catkin_ws/src
Build space: /home/amap/w_catkin_ws/build
Devel space: /home/amap/w_catkin_ws/devel
Install space: /home/amap/w_catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/amap/w_catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/amap/w_catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/amap/catkin_ws/devel;/opt/ros/melodic
-- This workspace overlays: /home/amap/catkin_ws/devel;/opt/ros/melodic
-- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python2
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/amap/w_catkin_ws/build/test_results
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python2 (found version "2.7.17") 
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.29
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 2 packages in topological order:
-- ~~  - car_control
-- ~~  - jetson_camera_rp_v2
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'car_control'
-- ==> add_subdirectory(car_control)
CMake Warning at /opt/ros/melodic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'system_lib' but neither
  'system_lib_INCLUDE_DIRS' nor 'system_lib_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/melodic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  car_control/CMakeLists.txt:27 (catkin_package)


-- +++ processing catkin package: 'jetson_camera_rp_v2'
-- ==> add_subdirectory(jetson_camera_rp_v2)
CMake Error at /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake:113 (message):
  Project 'cv_bridge' specifies '/usr/include/opencv' as an include dir,
  which is not found.  It does neither exist as an absolute directory nor in
  '${{prefix}}//usr/include/opencv'.  Check the issue tracker
  'https://github.com/ros-perception/vision_opencv/issues' and consider
  creating a ticket if the problem has not been reported yet.
Call Stack (most recent call first):
  /opt/ros/melodic/share/catkin/cmake/catkinConfig.cmake:76 (find_package)
  jetson_camera_rp_v2/CMakeLists.txt:15 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/amap/w_catkin_ws/build/CMakeFiles/CMakeOutput.log".
See also "/home/amap/w_catkin_ws/build/CMakeFiles/CMakeError.log".
Makefile:684: recipe for target 'cmake_check_build_system' failed
make: *** [cmake_check_build_system] Error 1
Invoking "make cmake_check_build_system" failed
