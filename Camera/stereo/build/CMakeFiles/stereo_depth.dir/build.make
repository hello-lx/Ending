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
CMAKE_SOURCE_DIR = /home/yuan/XSpace/Ending/Camera/stereo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuan/XSpace/Ending/Camera/stereo/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_depth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_depth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_depth.dir/flags.make

CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o: CMakeFiles/stereo_depth.dir/flags.make
CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o: ../src/my_stereo_depth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuan/XSpace/Ending/Camera/stereo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o -c /home/yuan/XSpace/Ending/Camera/stereo/src/my_stereo_depth.cpp

CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuan/XSpace/Ending/Camera/stereo/src/my_stereo_depth.cpp > CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.i

CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuan/XSpace/Ending/Camera/stereo/src/my_stereo_depth.cpp -o CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.s

CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.requires:

.PHONY : CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.requires

CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.provides: CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_depth.dir/build.make CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.provides

CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.provides.build: CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o


# Object files for target stereo_depth
stereo_depth_OBJECTS = \
"CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o"

# External object files for target stereo_depth
stereo_depth_EXTERNAL_OBJECTS =

stereo_depth: CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o
stereo_depth: CMakeFiles/stereo_depth.dir/build.make
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_stitching.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_superres.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_videostab.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_aruco.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_bgsegm.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_bioinspired.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_ccalib.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_cvv.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_dnn_objdetect.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_dpm.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_face.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_freetype.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_fuzzy.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_hfs.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_img_hash.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_line_descriptor.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_optflow.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_reg.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_rgbd.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_saliency.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_stereo.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_structured_light.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_surface_matching.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_tracking.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_xfeatures2d.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_ximgproc.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_xobjdetect.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_xphoto.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_shape.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_highgui.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_videoio.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_phase_unwrapping.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_video.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_datasets.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_plot.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_text.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_dnn.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_ml.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_imgcodecs.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_objdetect.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_calib3d.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_features2d.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_flann.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_photo.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_imgproc.so.3.4.9
stereo_depth: /home/yuan/Xstudio/Opencv/opencv-3.4.9/build/lib/libopencv_core.so.3.4.9
stereo_depth: CMakeFiles/stereo_depth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuan/XSpace/Ending/Camera/stereo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereo_depth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_depth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_depth.dir/build: stereo_depth

.PHONY : CMakeFiles/stereo_depth.dir/build

CMakeFiles/stereo_depth.dir/requires: CMakeFiles/stereo_depth.dir/src/my_stereo_depth.cpp.o.requires

.PHONY : CMakeFiles/stereo_depth.dir/requires

CMakeFiles/stereo_depth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_depth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_depth.dir/clean

CMakeFiles/stereo_depth.dir/depend:
	cd /home/yuan/XSpace/Ending/Camera/stereo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuan/XSpace/Ending/Camera/stereo /home/yuan/XSpace/Ending/Camera/stereo /home/yuan/XSpace/Ending/Camera/stereo/build /home/yuan/XSpace/Ending/Camera/stereo/build /home/yuan/XSpace/Ending/Camera/stereo/build/CMakeFiles/stereo_depth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_depth.dir/depend

