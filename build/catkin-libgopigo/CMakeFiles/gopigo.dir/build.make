# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/pi/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/catkin_ws/build

# Include any dependencies generated for this target.
include catkin-libgopigo/CMakeFiles/gopigo.dir/depend.make

# Include the progress variables for this target.
include catkin-libgopigo/CMakeFiles/gopigo.dir/progress.make

# Include the compile flags for this target's objects.
include catkin-libgopigo/CMakeFiles/gopigo.dir/flags.make

catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o: catkin-libgopigo/CMakeFiles/gopigo.dir/flags.make
catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o: /home/pi/catkin_ws/src/catkin-libgopigo/src/gopigo.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o"
	cd /home/pi/catkin_ws/build/catkin-libgopigo && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gopigo.dir/src/gopigo.c.o   -c /home/pi/catkin_ws/src/catkin-libgopigo/src/gopigo.c

catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gopigo.dir/src/gopigo.c.i"
	cd /home/pi/catkin_ws/build/catkin-libgopigo && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/catkin_ws/src/catkin-libgopigo/src/gopigo.c > CMakeFiles/gopigo.dir/src/gopigo.c.i

catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gopigo.dir/src/gopigo.c.s"
	cd /home/pi/catkin_ws/build/catkin-libgopigo && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/catkin_ws/src/catkin-libgopigo/src/gopigo.c -o CMakeFiles/gopigo.dir/src/gopigo.c.s

catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.requires:

.PHONY : catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.requires

catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.provides: catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.requires
	$(MAKE) -f catkin-libgopigo/CMakeFiles/gopigo.dir/build.make catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.provides.build
.PHONY : catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.provides

catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.provides.build: catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o


# Object files for target gopigo
gopigo_OBJECTS = \
"CMakeFiles/gopigo.dir/src/gopigo.c.o"

# External object files for target gopigo
gopigo_EXTERNAL_OBJECTS =

/home/pi/catkin_ws/devel/lib/libgopigo.so: catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o
/home/pi/catkin_ws/devel/lib/libgopigo.so: catkin-libgopigo/CMakeFiles/gopigo.dir/build.make
/home/pi/catkin_ws/devel/lib/libgopigo.so: catkin-libgopigo/CMakeFiles/gopigo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library /home/pi/catkin_ws/devel/lib/libgopigo.so"
	cd /home/pi/catkin_ws/build/catkin-libgopigo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gopigo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
catkin-libgopigo/CMakeFiles/gopigo.dir/build: /home/pi/catkin_ws/devel/lib/libgopigo.so

.PHONY : catkin-libgopigo/CMakeFiles/gopigo.dir/build

catkin-libgopigo/CMakeFiles/gopigo.dir/requires: catkin-libgopigo/CMakeFiles/gopigo.dir/src/gopigo.c.o.requires

.PHONY : catkin-libgopigo/CMakeFiles/gopigo.dir/requires

catkin-libgopigo/CMakeFiles/gopigo.dir/clean:
	cd /home/pi/catkin_ws/build/catkin-libgopigo && $(CMAKE_COMMAND) -P CMakeFiles/gopigo.dir/cmake_clean.cmake
.PHONY : catkin-libgopigo/CMakeFiles/gopigo.dir/clean

catkin-libgopigo/CMakeFiles/gopigo.dir/depend:
	cd /home/pi/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/catkin_ws/src /home/pi/catkin_ws/src/catkin-libgopigo /home/pi/catkin_ws/build /home/pi/catkin_ws/build/catkin-libgopigo /home/pi/catkin_ws/build/catkin-libgopigo/CMakeFiles/gopigo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : catkin-libgopigo/CMakeFiles/gopigo.dir/depend

