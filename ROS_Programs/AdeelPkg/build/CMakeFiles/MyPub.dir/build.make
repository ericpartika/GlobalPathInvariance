# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/build

# Include any dependencies generated for this target.
include CMakeFiles/MyPub.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MyPub.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MyPub.dir/flags.make

CMakeFiles/MyPub.dir/src/pub.cpp.o: CMakeFiles/MyPub.dir/flags.make
CMakeFiles/MyPub.dir/src/pub.cpp.o: ../src/pub.cpp
CMakeFiles/MyPub.dir/src/pub.cpp.o: ../manifest.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/catkin/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/rospack/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/roslib/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/MyPub.dir/src/pub.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/MyPub.dir/src/pub.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/MyPub.dir/src/pub.cpp.o -c /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/src/pub.cpp

CMakeFiles/MyPub.dir/src/pub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MyPub.dir/src/pub.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/src/pub.cpp > CMakeFiles/MyPub.dir/src/pub.cpp.i

CMakeFiles/MyPub.dir/src/pub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MyPub.dir/src/pub.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/src/pub.cpp -o CMakeFiles/MyPub.dir/src/pub.cpp.s

CMakeFiles/MyPub.dir/src/pub.cpp.o.requires:
.PHONY : CMakeFiles/MyPub.dir/src/pub.cpp.o.requires

CMakeFiles/MyPub.dir/src/pub.cpp.o.provides: CMakeFiles/MyPub.dir/src/pub.cpp.o.requires
	$(MAKE) -f CMakeFiles/MyPub.dir/build.make CMakeFiles/MyPub.dir/src/pub.cpp.o.provides.build
.PHONY : CMakeFiles/MyPub.dir/src/pub.cpp.o.provides

CMakeFiles/MyPub.dir/src/pub.cpp.o.provides.build: CMakeFiles/MyPub.dir/src/pub.cpp.o

# Object files for target MyPub
MyPub_OBJECTS = \
"CMakeFiles/MyPub.dir/src/pub.cpp.o"

# External object files for target MyPub
MyPub_EXTERNAL_OBJECTS =

../bin/MyPub: CMakeFiles/MyPub.dir/src/pub.cpp.o
../bin/MyPub: CMakeFiles/MyPub.dir/build.make
../bin/MyPub: CMakeFiles/MyPub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/MyPub"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MyPub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MyPub.dir/build: ../bin/MyPub
.PHONY : CMakeFiles/MyPub.dir/build

CMakeFiles/MyPub.dir/requires: CMakeFiles/MyPub.dir/src/pub.cpp.o.requires
.PHONY : CMakeFiles/MyPub.dir/requires

CMakeFiles/MyPub.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MyPub.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MyPub.dir/clean

CMakeFiles/MyPub.dir/depend:
	cd /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/build /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/build /home/a5akhtar/AdeelAkhtar/PhDstuff/ROS_Programs/AdeelPkg/build/CMakeFiles/MyPub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MyPub.dir/depend
