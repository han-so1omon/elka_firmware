# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_COMMAND = /opt/cmake-3.4.3-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.4.3-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka

# Include any dependencies generated for this target.
include CMakeFiles/elka.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/elka.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/elka.dir/flags.make

CMakeFiles/elka.dir/src/main.c.obj: CMakeFiles/elka.dir/flags.make
CMakeFiles/elka.dir/src/main.c.obj: ../src/main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/elka.dir/src/main.c.obj"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/elka.dir/src/main.c.obj   -c /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/main.c

CMakeFiles/elka.dir/src/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elka.dir/src/main.c.i"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/main.c > CMakeFiles/elka.dir/src/main.c.i

CMakeFiles/elka.dir/src/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elka.dir/src/main.c.s"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/main.c -o CMakeFiles/elka.dir/src/main.c.s

CMakeFiles/elka.dir/src/main.c.obj.requires:

.PHONY : CMakeFiles/elka.dir/src/main.c.obj.requires

CMakeFiles/elka.dir/src/main.c.obj.provides: CMakeFiles/elka.dir/src/main.c.obj.requires
	$(MAKE) -f CMakeFiles/elka.dir/build.make CMakeFiles/elka.dir/src/main.c.obj.provides.build
.PHONY : CMakeFiles/elka.dir/src/main.c.obj.provides

CMakeFiles/elka.dir/src/main.c.obj.provides.build: CMakeFiles/elka.dir/src/main.c.obj


CMakeFiles/elka.dir/src/_write.c.obj: CMakeFiles/elka.dir/flags.make
CMakeFiles/elka.dir/src/_write.c.obj: ../src/_write.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/elka.dir/src/_write.c.obj"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/elka.dir/src/_write.c.obj   -c /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/_write.c

CMakeFiles/elka.dir/src/_write.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elka.dir/src/_write.c.i"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/_write.c > CMakeFiles/elka.dir/src/_write.c.i

CMakeFiles/elka.dir/src/_write.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elka.dir/src/_write.c.s"
	/usr/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/src/_write.c -o CMakeFiles/elka.dir/src/_write.c.s

CMakeFiles/elka.dir/src/_write.c.obj.requires:

.PHONY : CMakeFiles/elka.dir/src/_write.c.obj.requires

CMakeFiles/elka.dir/src/_write.c.obj.provides: CMakeFiles/elka.dir/src/_write.c.obj.requires
	$(MAKE) -f CMakeFiles/elka.dir/build.make CMakeFiles/elka.dir/src/_write.c.obj.provides.build
.PHONY : CMakeFiles/elka.dir/src/_write.c.obj.provides

CMakeFiles/elka.dir/src/_write.c.obj.provides.build: CMakeFiles/elka.dir/src/_write.c.obj


# Object files for target elka
elka_OBJECTS = \
"CMakeFiles/elka.dir/src/main.c.obj" \
"CMakeFiles/elka.dir/src/_write.c.obj"

# External object files for target elka
elka_EXTERNAL_OBJECTS =

elka: CMakeFiles/elka.dir/src/main.c.obj
elka: CMakeFiles/elka.dir/src/_write.c.obj
elka: CMakeFiles/elka.dir/build.make
elka: CMakeFiles/elka.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable elka"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/elka.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/elka.dir/build: elka

.PHONY : CMakeFiles/elka.dir/build

CMakeFiles/elka.dir/requires: CMakeFiles/elka.dir/src/main.c.obj.requires
CMakeFiles/elka.dir/requires: CMakeFiles/elka.dir/src/_write.c.obj.requires

.PHONY : CMakeFiles/elka.dir/requires

CMakeFiles/elka.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/elka.dir/cmake_clean.cmake
.PHONY : CMakeFiles/elka.dir/clean

CMakeFiles/elka.dir/depend:
	cd /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka /home/eric/Programs/elka/elka_firmware/elka_stm32f4_cmake/build_elka/CMakeFiles/elka.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/elka.dir/depend

