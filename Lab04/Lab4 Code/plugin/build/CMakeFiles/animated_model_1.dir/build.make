# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/meam520/Lab4/plugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/meam520/Lab4/plugin/build

# Include any dependencies generated for this target.
include CMakeFiles/animated_model_1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/animated_model_1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/animated_model_1.dir/flags.make

CMakeFiles/animated_model_1.dir/animated_model_1.cc.o: CMakeFiles/animated_model_1.dir/flags.make
CMakeFiles/animated_model_1.dir/animated_model_1.cc.o: ../animated_model_1.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/meam520/Lab4/plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/animated_model_1.dir/animated_model_1.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/animated_model_1.dir/animated_model_1.cc.o -c /home/meam520/Lab4/plugin/animated_model_1.cc

CMakeFiles/animated_model_1.dir/animated_model_1.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/animated_model_1.dir/animated_model_1.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/meam520/Lab4/plugin/animated_model_1.cc > CMakeFiles/animated_model_1.dir/animated_model_1.cc.i

CMakeFiles/animated_model_1.dir/animated_model_1.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/animated_model_1.dir/animated_model_1.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/meam520/Lab4/plugin/animated_model_1.cc -o CMakeFiles/animated_model_1.dir/animated_model_1.cc.s

CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.requires:

.PHONY : CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.requires

CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.provides: CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.requires
	$(MAKE) -f CMakeFiles/animated_model_1.dir/build.make CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.provides.build
.PHONY : CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.provides

CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.provides.build: CMakeFiles/animated_model_1.dir/animated_model_1.cc.o


# Object files for target animated_model_1
animated_model_1_OBJECTS = \
"CMakeFiles/animated_model_1.dir/animated_model_1.cc.o"

# External object files for target animated_model_1
animated_model_1_EXTERNAL_OBJECTS =

libanimated_model_1.so: CMakeFiles/animated_model_1.dir/animated_model_1.cc.o
libanimated_model_1.so: CMakeFiles/animated_model_1.dir/build.make
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libblas.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libblas.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libignition-transport4.so.4.0.0
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libignition-common1.so.1.0.1
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.0.0
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libanimated_model_1.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libanimated_model_1.so: CMakeFiles/animated_model_1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/meam520/Lab4/plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libanimated_model_1.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/animated_model_1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/animated_model_1.dir/build: libanimated_model_1.so

.PHONY : CMakeFiles/animated_model_1.dir/build

CMakeFiles/animated_model_1.dir/requires: CMakeFiles/animated_model_1.dir/animated_model_1.cc.o.requires

.PHONY : CMakeFiles/animated_model_1.dir/requires

CMakeFiles/animated_model_1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/animated_model_1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/animated_model_1.dir/clean

CMakeFiles/animated_model_1.dir/depend:
	cd /home/meam520/Lab4/plugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/meam520/Lab4/plugin /home/meam520/Lab4/plugin /home/meam520/Lab4/plugin/build /home/meam520/Lab4/plugin/build /home/meam520/Lab4/plugin/build/CMakeFiles/animated_model_1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/animated_model_1.dir/depend
