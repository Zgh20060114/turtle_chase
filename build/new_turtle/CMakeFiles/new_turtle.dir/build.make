# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zgh/ROS_Projects/turtle_chase/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zgh/ROS_Projects/turtle_chase/build

# Include any dependencies generated for this target.
include new_turtle/CMakeFiles/new_turtle.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include new_turtle/CMakeFiles/new_turtle.dir/compiler_depend.make

# Include the progress variables for this target.
include new_turtle/CMakeFiles/new_turtle.dir/progress.make

# Include the compile flags for this target's objects.
include new_turtle/CMakeFiles/new_turtle.dir/flags.make

new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o: new_turtle/CMakeFiles/new_turtle.dir/flags.make
new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o: /home/zgh/ROS_Projects/turtle_chase/src/new_turtle/src/new_turtle.cpp
new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o: new_turtle/CMakeFiles/new_turtle.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgh/ROS_Projects/turtle_chase/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o"
	cd /home/zgh/ROS_Projects/turtle_chase/build/new_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o -MF CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o.d -o CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o -c /home/zgh/ROS_Projects/turtle_chase/src/new_turtle/src/new_turtle.cpp

new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/new_turtle.dir/src/new_turtle.cpp.i"
	cd /home/zgh/ROS_Projects/turtle_chase/build/new_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zgh/ROS_Projects/turtle_chase/src/new_turtle/src/new_turtle.cpp > CMakeFiles/new_turtle.dir/src/new_turtle.cpp.i

new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/new_turtle.dir/src/new_turtle.cpp.s"
	cd /home/zgh/ROS_Projects/turtle_chase/build/new_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zgh/ROS_Projects/turtle_chase/src/new_turtle/src/new_turtle.cpp -o CMakeFiles/new_turtle.dir/src/new_turtle.cpp.s

# Object files for target new_turtle
new_turtle_OBJECTS = \
"CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o"

# External object files for target new_turtle
new_turtle_EXTERNAL_OBJECTS =

/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: new_turtle/CMakeFiles/new_turtle.dir/src/new_turtle.cpp.o
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: new_turtle/CMakeFiles/new_turtle.dir/build.make
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/libroscpp.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/librosconsole.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/librostime.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /opt/ros/noetic/lib/libcpp_common.so
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle: new_turtle/CMakeFiles/new_turtle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zgh/ROS_Projects/turtle_chase/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle"
	cd /home/zgh/ROS_Projects/turtle_chase/build/new_turtle && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/new_turtle.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
new_turtle/CMakeFiles/new_turtle.dir/build: /home/zgh/ROS_Projects/turtle_chase/devel/lib/new_turtle/new_turtle
.PHONY : new_turtle/CMakeFiles/new_turtle.dir/build

new_turtle/CMakeFiles/new_turtle.dir/clean:
	cd /home/zgh/ROS_Projects/turtle_chase/build/new_turtle && $(CMAKE_COMMAND) -P CMakeFiles/new_turtle.dir/cmake_clean.cmake
.PHONY : new_turtle/CMakeFiles/new_turtle.dir/clean

new_turtle/CMakeFiles/new_turtle.dir/depend:
	cd /home/zgh/ROS_Projects/turtle_chase/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zgh/ROS_Projects/turtle_chase/src /home/zgh/ROS_Projects/turtle_chase/src/new_turtle /home/zgh/ROS_Projects/turtle_chase/build /home/zgh/ROS_Projects/turtle_chase/build/new_turtle /home/zgh/ROS_Projects/turtle_chase/build/new_turtle/CMakeFiles/new_turtle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : new_turtle/CMakeFiles/new_turtle.dir/depend

