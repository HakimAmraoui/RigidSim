# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build

# Include any dependencies generated for this target.
include dep/glfw/tests/CMakeFiles/reopen.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include dep/glfw/tests/CMakeFiles/reopen.dir/compiler_depend.make

# Include the progress variables for this target.
include dep/glfw/tests/CMakeFiles/reopen.dir/progress.make

# Include the compile flags for this target's objects.
include dep/glfw/tests/CMakeFiles/reopen.dir/flags.make

dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.o: dep/glfw/tests/CMakeFiles/reopen.dir/flags.make
dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.o: ../dep/glfw/tests/reopen.c
dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.o: dep/glfw/tests/CMakeFiles/reopen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.o"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.o -MF CMakeFiles/reopen.dir/reopen.c.o.d -o CMakeFiles/reopen.dir/reopen.c.o -c /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/tests/reopen.c

dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/reopen.dir/reopen.c.i"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/tests/reopen.c > CMakeFiles/reopen.dir/reopen.c.i

dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/reopen.dir/reopen.c.s"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/tests/reopen.c -o CMakeFiles/reopen.dir/reopen.c.s

dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.o: dep/glfw/tests/CMakeFiles/reopen.dir/flags.make
dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.o: ../dep/glfw/deps/glad_gl.c
dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.o: dep/glfw/tests/CMakeFiles/reopen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.o"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.o -MF CMakeFiles/reopen.dir/__/deps/glad_gl.c.o.d -o CMakeFiles/reopen.dir/__/deps/glad_gl.c.o -c /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/deps/glad_gl.c

dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/reopen.dir/__/deps/glad_gl.c.i"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/deps/glad_gl.c > CMakeFiles/reopen.dir/__/deps/glad_gl.c.i

dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/reopen.dir/__/deps/glad_gl.c.s"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/deps/glad_gl.c -o CMakeFiles/reopen.dir/__/deps/glad_gl.c.s

# Object files for target reopen
reopen_OBJECTS = \
"CMakeFiles/reopen.dir/reopen.c.o" \
"CMakeFiles/reopen.dir/__/deps/glad_gl.c.o"

# External object files for target reopen
reopen_EXTERNAL_OBJECTS =

dep/glfw/tests/reopen: dep/glfw/tests/CMakeFiles/reopen.dir/reopen.c.o
dep/glfw/tests/reopen: dep/glfw/tests/CMakeFiles/reopen.dir/__/deps/glad_gl.c.o
dep/glfw/tests/reopen: dep/glfw/tests/CMakeFiles/reopen.dir/build.make
dep/glfw/tests/reopen: dep/glfw/src/libglfw3.a
dep/glfw/tests/reopen: /usr/lib/libm.so
dep/glfw/tests/reopen: /usr/lib/librt.so
dep/glfw/tests/reopen: /usr/lib/libm.so
dep/glfw/tests/reopen: /usr/lib/libX11.so
dep/glfw/tests/reopen: dep/glfw/tests/CMakeFiles/reopen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable reopen"
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reopen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dep/glfw/tests/CMakeFiles/reopen.dir/build: dep/glfw/tests/reopen
.PHONY : dep/glfw/tests/CMakeFiles/reopen.dir/build

dep/glfw/tests/CMakeFiles/reopen.dir/clean:
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests && $(CMAKE_COMMAND) -P CMakeFiles/reopen.dir/cmake_clean.cmake
.PHONY : dep/glfw/tests/CMakeFiles/reopen.dir/clean

dep/glfw/tests/CMakeFiles/reopen.dir/depend:
	cd /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/dep/glfw/tests /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests /home/akimh/Documents/Master/M1/S2/IG3D/TME/RigidSim/build/dep/glfw/tests/CMakeFiles/reopen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dep/glfw/tests/CMakeFiles/reopen.dir/depend
