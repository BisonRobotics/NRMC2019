# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/fworg64/NRMC2019/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fworg64/NRMC2019/src

# Include any dependencies generated for this target.
include mapping/PC2CM/CMakeFiles/pc2_processor.dir/depend.make

# Include the progress variables for this target.
include mapping/PC2CM/CMakeFiles/pc2_processor.dir/progress.make

# Include the compile flags for this target's objects.
include mapping/PC2CM/CMakeFiles/pc2_processor.dir/flags.make

mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o: mapping/PC2CM/CMakeFiles/pc2_processor.dir/flags.make
mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o: mapping/PC2CM/src/pc2_processor/pc2_processor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fworg64/NRMC2019/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o"
	cd /home/fworg64/NRMC2019/src/mapping/PC2CM && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o -c /home/fworg64/NRMC2019/src/mapping/PC2CM/src/pc2_processor/pc2_processor.cpp

mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.i"
	cd /home/fworg64/NRMC2019/src/mapping/PC2CM && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fworg64/NRMC2019/src/mapping/PC2CM/src/pc2_processor/pc2_processor.cpp > CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.i

mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.s"
	cd /home/fworg64/NRMC2019/src/mapping/PC2CM && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fworg64/NRMC2019/src/mapping/PC2CM/src/pc2_processor/pc2_processor.cpp -o CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.s

mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.requires:

.PHONY : mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.requires

mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.provides: mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.requires
	$(MAKE) -f mapping/PC2CM/CMakeFiles/pc2_processor.dir/build.make mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.provides.build
.PHONY : mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.provides

mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.provides.build: mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o


# Object files for target pc2_processor
pc2_processor_OBJECTS = \
"CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o"

# External object files for target pc2_processor
pc2_processor_EXTERNAL_OBJECTS =

/home/fworg64/NRMC2019/devel/lib/libpc2_processor.so: mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o
/home/fworg64/NRMC2019/devel/lib/libpc2_processor.so: mapping/PC2CM/CMakeFiles/pc2_processor.dir/build.make
/home/fworg64/NRMC2019/devel/lib/libpc2_processor.so: mapping/PC2CM/CMakeFiles/pc2_processor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fworg64/NRMC2019/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/fworg64/NRMC2019/devel/lib/libpc2_processor.so"
	cd /home/fworg64/NRMC2019/src/mapping/PC2CM && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pc2_processor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mapping/PC2CM/CMakeFiles/pc2_processor.dir/build: /home/fworg64/NRMC2019/devel/lib/libpc2_processor.so

.PHONY : mapping/PC2CM/CMakeFiles/pc2_processor.dir/build

mapping/PC2CM/CMakeFiles/pc2_processor.dir/requires: mapping/PC2CM/CMakeFiles/pc2_processor.dir/src/pc2_processor/pc2_processor.cpp.o.requires

.PHONY : mapping/PC2CM/CMakeFiles/pc2_processor.dir/requires

mapping/PC2CM/CMakeFiles/pc2_processor.dir/clean:
	cd /home/fworg64/NRMC2019/src/mapping/PC2CM && $(CMAKE_COMMAND) -P CMakeFiles/pc2_processor.dir/cmake_clean.cmake
.PHONY : mapping/PC2CM/CMakeFiles/pc2_processor.dir/clean

mapping/PC2CM/CMakeFiles/pc2_processor.dir/depend:
	cd /home/fworg64/NRMC2019/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fworg64/NRMC2019/src /home/fworg64/NRMC2019/src/mapping/PC2CM /home/fworg64/NRMC2019/src /home/fworg64/NRMC2019/src/mapping/PC2CM /home/fworg64/NRMC2019/src/mapping/PC2CM/CMakeFiles/pc2_processor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mapping/PC2CM/CMakeFiles/pc2_processor.dir/depend

