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
CMAKE_SOURCE_DIR = /home/felipe-narvaez/workspace/plainDrawer/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/felipe-narvaez/workspace/plainDrawer/build

# Include any dependencies generated for this target.
include CMakeFiles/plainDrawer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/plainDrawer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plainDrawer.dir/flags.make

CMakeFiles/plainDrawer.dir/main.cpp.o: CMakeFiles/plainDrawer.dir/flags.make
CMakeFiles/plainDrawer.dir/main.cpp.o: /home/felipe-narvaez/workspace/plainDrawer/src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/felipe-narvaez/workspace/plainDrawer/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/plainDrawer.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/plainDrawer.dir/main.cpp.o -c /home/felipe-narvaez/workspace/plainDrawer/src/main.cpp

CMakeFiles/plainDrawer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plainDrawer.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/felipe-narvaez/workspace/plainDrawer/src/main.cpp > CMakeFiles/plainDrawer.dir/main.cpp.i

CMakeFiles/plainDrawer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plainDrawer.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/felipe-narvaez/workspace/plainDrawer/src/main.cpp -o CMakeFiles/plainDrawer.dir/main.cpp.s

CMakeFiles/plainDrawer.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/plainDrawer.dir/main.cpp.o.requires

CMakeFiles/plainDrawer.dir/main.cpp.o.provides: CMakeFiles/plainDrawer.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/plainDrawer.dir/build.make CMakeFiles/plainDrawer.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/plainDrawer.dir/main.cpp.o.provides

CMakeFiles/plainDrawer.dir/main.cpp.o.provides.build: CMakeFiles/plainDrawer.dir/main.cpp.o

CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o: CMakeFiles/plainDrawer.dir/flags.make
CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o: /home/felipe-narvaez/workspace/plainDrawer/src/utils/Utilities.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/felipe-narvaez/workspace/plainDrawer/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o -c /home/felipe-narvaez/workspace/plainDrawer/src/utils/Utilities.cpp

CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/felipe-narvaez/workspace/plainDrawer/src/utils/Utilities.cpp > CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.i

CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/felipe-narvaez/workspace/plainDrawer/src/utils/Utilities.cpp -o CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.s

CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.requires:
.PHONY : CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.requires

CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.provides: CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.requires
	$(MAKE) -f CMakeFiles/plainDrawer.dir/build.make CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.provides.build
.PHONY : CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.provides

CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.provides.build: CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o

CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o: CMakeFiles/plainDrawer.dir/flags.make
CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o: /home/felipe-narvaez/workspace/plainDrawer/src/utils/Viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/felipe-narvaez/workspace/plainDrawer/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o -c /home/felipe-narvaez/workspace/plainDrawer/src/utils/Viewer.cpp

CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/felipe-narvaez/workspace/plainDrawer/src/utils/Viewer.cpp > CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.i

CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/felipe-narvaez/workspace/plainDrawer/src/utils/Viewer.cpp -o CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.s

CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.requires:
.PHONY : CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.requires

CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.provides: CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/plainDrawer.dir/build.make CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.provides.build
.PHONY : CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.provides

CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.provides.build: CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o

# Object files for target plainDrawer
plainDrawer_OBJECTS = \
"CMakeFiles/plainDrawer.dir/main.cpp.o" \
"CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o" \
"CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o"

# External object files for target plainDrawer
plainDrawer_EXTERNAL_OBJECTS =

plainDrawer: CMakeFiles/plainDrawer.dir/main.cpp.o
plainDrawer: CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o
plainDrawer: CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o
plainDrawer: CMakeFiles/plainDrawer.dir/build.make
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_system.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libpthread.so
plainDrawer: /usr/local/lib/libpcl_common.so
plainDrawer: /usr/local/lib/libpcl_octree.so
plainDrawer: /usr/lib/libOpenNI.so
plainDrawer: /usr/local/lib/libvtksys-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonSystem-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkproj4-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersTexture-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeneral-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonComputationalGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonDataModel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonMath-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonMisc-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonTransforms-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonExecutionModel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingStencil-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersStatistics-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingFourier-6.2.so.1
plainDrawer: /usr/local/lib/libvtkalglib-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionStyle-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersExtraction-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSources-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonColor-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkpng-6.2.so.1
plainDrawer: /usr/local/lib/libvtkzlib-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingAnnotation-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingColor-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingFreeType-6.2.so.1
plainDrawer: /usr/local/lib/libvtkfreetype-6.2.so.1
plainDrawer: /usr/local/lib/libvtkftgl-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingGeneral-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingSources-6.2.so.1
plainDrawer: /usr/local/lib/libvtksqlite-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOExport-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkDICOMParser-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkmetaio-6.2.so.1
plainDrawer: /usr/local/lib/libvtkjpeg-6.2.so.1
plainDrawer: /usr/local/lib/libvtktiff-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingContext2D-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingGL2PS-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingContextOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingHybrid-6.2.so.1
plainDrawer: /usr/local/lib/libvtkgl2ps-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLabel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkhdf5_hl-6.2.so.1
plainDrawer: /usr/local/lib/libvtkhdf5-6.2.so.1
plainDrawer: /usr/local/lib/libvtkjsoncpp-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLOD-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersModeling-6.2.so.1
plainDrawer: /usr/local/lib/libvtkDomainsChemistry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOXML-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOXMLParser-6.2.so.1
plainDrawer: /usr/local/lib/libvtkexpat-6.2.so.1
plainDrawer: /usr/local/lib/libvtkNetCDF-6.2.so.1
plainDrawer: /usr/local/lib/libvtkNetCDF_cxx-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsContext2D-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionWidgets-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersHybrid-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingVolume-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingFreeTypeOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersProgrammable-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQtOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersVerdict-6.2.so.1
plainDrawer: /usr/local/lib/libvtkverdict-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOPLY-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOParallelXML-6.2.so.1
plainDrawer: /usr/local/lib/libvtkParallelCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOLegacy-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkChartsCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInfovisCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQtSQL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOSQL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersAMR-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersHyperTree-6.2.so.1
plainDrawer: /usr/local/lib/libvtkoggtheora-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOAMR-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeneric-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOImport-6.2.so.1
plainDrawer: /usr/local/lib/libvtklibxml2-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQtWebkit-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsInfovis-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersImaging-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInfovisLayout-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGeovisCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersParallelImaging-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersParallel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOVideo-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOLSDyna-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOInfovis-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingMath-6.2.so.1
plainDrawer: /usr/local/lib/libvtkexoIIc-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOExodus-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersFlowPaths-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOMovie-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSMP-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSelection-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingStatistics-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOEnSight-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLIC-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIONetCDF-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOMINC-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingMorphological-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOParallel-6.2.so.1
plainDrawer: /usr/local/lib/libpcl_io.so
plainDrawer: /usr/local/lib/libpcl_gpu_containers.so
plainDrawer: /usr/local/lib/libpcl_gpu_utils.so
plainDrawer: /usr/local/lib/libpcl_gpu_people.so
plainDrawer: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
plainDrawer: /usr/local/lib/libpcl_gpu_surface.so
plainDrawer: /usr/local/lib/libpcl_gpu_octree.so
plainDrawer: /usr/local/lib/libpcl_gpu_kinfu.so
plainDrawer: /usr/local/lib/libpcl_gpu_segmentation.so
plainDrawer: /usr/local/lib/libpcl_gpu_features.so
plainDrawer: /usr/local/lib/libpcl_ml.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
plainDrawer: /usr/local/lib/libpcl_kdtree.so
plainDrawer: /usr/local/lib/libpcl_search.so
plainDrawer: /usr/local/lib/libpcl_sample_consensus.so
plainDrawer: /usr/local/lib/libpcl_filters.so
plainDrawer: /usr/local/lib/libpcl_features.so
plainDrawer: /usr/local/lib/libpcl_keypoints.so
plainDrawer: /usr/local/lib/libpcl_segmentation.so
plainDrawer: /usr/local/lib/libpcl_visualization.so
plainDrawer: /usr/local/lib/libpcl_outofcore.so
plainDrawer: /usr/local/lib/libpcl_stereo.so
plainDrawer: /usr/local/lib/libpcl_people.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libqhull.so
plainDrawer: /usr/local/lib/libpcl_surface.so
plainDrawer: /usr/local/lib/libpcl_registration.so
plainDrawer: /usr/local/lib/libpcl_cuda_io.so
plainDrawer: /usr/local/lib/libpcl_cuda_sample_consensus.so
plainDrawer: /usr/local/lib/libpcl_cuda_features.so
plainDrawer: /usr/local/lib/libpcl_cuda_segmentation.so
plainDrawer: /usr/local/lib/libpcl_tracking.so
plainDrawer: /usr/local/lib/libpcl_recognition.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_system.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libpthread.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libqhull.so
plainDrawer: /usr/lib/libOpenNI.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
plainDrawer: /usr/local/lib/libvtksys-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonSystem-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkproj4-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersTexture-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeneral-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonComputationalGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonDataModel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonMath-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonMisc-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonTransforms-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonExecutionModel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingStencil-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersStatistics-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingFourier-6.2.so.1
plainDrawer: /usr/local/lib/libvtkalglib-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionStyle-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersExtraction-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSources-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonColor-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkpng-6.2.so.1
plainDrawer: /usr/local/lib/libvtkzlib-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingAnnotation-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingColor-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingFreeType-6.2.so.1
plainDrawer: /usr/local/lib/libvtkfreetype-6.2.so.1
plainDrawer: /usr/local/lib/libvtkftgl-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingGeneral-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingSources-6.2.so.1
plainDrawer: /usr/local/lib/libvtksqlite-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOExport-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkDICOMParser-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkmetaio-6.2.so.1
plainDrawer: /usr/local/lib/libvtkjpeg-6.2.so.1
plainDrawer: /usr/local/lib/libvtktiff-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingContext2D-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingGL2PS-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingContextOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingHybrid-6.2.so.1
plainDrawer: /usr/local/lib/libvtkgl2ps-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLabel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkhdf5_hl-6.2.so.1
plainDrawer: /usr/local/lib/libvtkhdf5-6.2.so.1
plainDrawer: /usr/local/lib/libvtkjsoncpp-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLOD-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersModeling-6.2.so.1
plainDrawer: /usr/local/lib/libvtkDomainsChemistry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOXML-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOXMLParser-6.2.so.1
plainDrawer: /usr/local/lib/libvtkexpat-6.2.so.1
plainDrawer: /usr/local/lib/libvtkNetCDF-6.2.so.1
plainDrawer: /usr/local/lib/libvtkNetCDF_cxx-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsContext2D-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionWidgets-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersHybrid-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingVolume-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingFreeTypeOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersProgrammable-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQtOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersVerdict-6.2.so.1
plainDrawer: /usr/local/lib/libvtkverdict-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOPLY-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOParallelXML-6.2.so.1
plainDrawer: /usr/local/lib/libvtkParallelCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOLegacy-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkChartsCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInfovisCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQtSQL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOSQL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersAMR-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersHyperTree-6.2.so.1
plainDrawer: /usr/local/lib/libvtkoggtheora-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOAMR-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeneric-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOImport-6.2.so.1
plainDrawer: /usr/local/lib/libvtklibxml2-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQtWebkit-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsInfovis-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersImaging-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInfovisLayout-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGeovisCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersParallelImaging-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersParallel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOVideo-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOLSDyna-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOInfovis-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingMath-6.2.so.1
plainDrawer: /usr/local/lib/libvtkexoIIc-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOExodus-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersFlowPaths-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOMovie-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSMP-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSelection-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingStatistics-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOEnSight-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLIC-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIONetCDF-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOMINC-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingMorphological-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOParallel-6.2.so.1
plainDrawer: /usr/local/lib/libpcl_common.so
plainDrawer: /usr/local/lib/libpcl_octree.so
plainDrawer: /usr/local/lib/libpcl_io.so
plainDrawer: /usr/local/lib/libpcl_gpu_containers.so
plainDrawer: /usr/local/lib/libpcl_gpu_utils.so
plainDrawer: /usr/local/lib/libpcl_gpu_people.so
plainDrawer: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
plainDrawer: /usr/local/lib/libpcl_gpu_surface.so
plainDrawer: /usr/local/lib/libpcl_gpu_octree.so
plainDrawer: /usr/local/lib/libpcl_gpu_kinfu.so
plainDrawer: /usr/local/lib/libpcl_gpu_segmentation.so
plainDrawer: /usr/local/lib/libpcl_gpu_features.so
plainDrawer: /usr/local/lib/libpcl_ml.so
plainDrawer: /usr/local/lib/libpcl_kdtree.so
plainDrawer: /usr/local/lib/libpcl_search.so
plainDrawer: /usr/local/lib/libpcl_sample_consensus.so
plainDrawer: /usr/local/lib/libpcl_filters.so
plainDrawer: /usr/local/lib/libpcl_features.so
plainDrawer: /usr/local/lib/libpcl_keypoints.so
plainDrawer: /usr/local/lib/libpcl_segmentation.so
plainDrawer: /usr/local/lib/libpcl_visualization.so
plainDrawer: /usr/local/lib/libpcl_outofcore.so
plainDrawer: /usr/local/lib/libpcl_stereo.so
plainDrawer: /usr/local/lib/libpcl_people.so
plainDrawer: /usr/local/lib/libpcl_surface.so
plainDrawer: /usr/local/lib/libpcl_registration.so
plainDrawer: /usr/local/lib/libpcl_cuda_io.so
plainDrawer: /usr/local/lib/libpcl_cuda_sample_consensus.so
plainDrawer: /usr/local/lib/libpcl_cuda_features.so
plainDrawer: /usr/local/lib/libpcl_cuda_segmentation.so
plainDrawer: /usr/local/lib/libpcl_tracking.so
plainDrawer: /usr/local/lib/libpcl_recognition.so
plainDrawer: /usr/local/lib/libvtksqlite-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersAMR-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersTexture-6.2.so.1
plainDrawer: /usr/local/lib/libvtkGUISupportQt-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingLabel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkChartsCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingContext2D-6.2.so.1
plainDrawer: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.2.1
plainDrawer: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.2.1
plainDrawer: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.2.1
plainDrawer: /usr/local/lib/libvtkproj4-6.2.so.1
plainDrawer: /usr/local/lib/libvtkViewsCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInfovisLayout-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersImaging-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInfovisCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtklibxml2-6.2.so.1
plainDrawer: /usr/local/lib/libvtkoggtheora-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingOpenGL-6.2.so.1
plainDrawer: /usr/lib/x86_64-linux-gnu/libGLU.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libSM.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libICE.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libX11.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libXext.so
plainDrawer: /usr/lib/x86_64-linux-gnu/libXt.so
plainDrawer: /usr/local/lib/libvtkInteractionWidgets-6.2.so.1
plainDrawer: /usr/local/lib/libvtkInteractionStyle-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingAnnotation-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingColor-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingFreeType-6.2.so.1
plainDrawer: /usr/local/lib/libvtkftgl-6.2.so.1
plainDrawer: /usr/local/lib/libvtkfreetype-6.2.so.1
plainDrawer: /usr/lib/x86_64-linux-gnu/libGL.so
plainDrawer: /usr/local/lib/libvtkImagingHybrid-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingVolume-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersHybrid-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingGeneral-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingSources-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOImage-6.2.so.1
plainDrawer: /usr/local/lib/libvtkpng-6.2.so.1
plainDrawer: /usr/local/lib/libvtkDICOMParser-6.2.so.1
plainDrawer: /usr/local/lib/libvtkmetaio-6.2.so.1
plainDrawer: /usr/local/lib/libvtktiff-6.2.so.1
plainDrawer: /usr/local/lib/libvtkjpeg-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOXML-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkjsoncpp-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOXMLParser-6.2.so.1
plainDrawer: /usr/local/lib/libvtkexpat-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersParallel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkRenderingCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersExtraction-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersStatistics-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingFourier-6.2.so.1
plainDrawer: /usr/local/lib/libvtkImagingCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkalglib-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonColor-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersModeling-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersSources-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersGeneral-6.2.so.1
plainDrawer: /usr/local/lib/libvtkFiltersCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonComputationalGeometry-6.2.so.1
plainDrawer: /usr/local/lib/libvtkParallelCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOLegacy-6.2.so.1
plainDrawer: /usr/local/lib/libvtkexoIIc-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIONetCDF-6.2.so.1
plainDrawer: /usr/local/lib/libvtkIOCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonExecutionModel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonDataModel-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonSystem-6.2.so.1
plainDrawer: /usr/local/lib/libvtksys-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonMisc-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonTransforms-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonMath-6.2.so.1
plainDrawer: /usr/local/lib/libvtkCommonCore-6.2.so.1
plainDrawer: /usr/local/lib/libvtkNetCDF_cxx-6.2.so.1
plainDrawer: /usr/local/lib/libvtkNetCDF-6.2.so.1
plainDrawer: /usr/local/lib/libvtkhdf5_hl-6.2.so.1
plainDrawer: /usr/local/lib/libvtkhdf5-6.2.so.1
plainDrawer: /usr/local/lib/libvtkzlib-6.2.so.1
plainDrawer: CMakeFiles/plainDrawer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable plainDrawer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plainDrawer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plainDrawer.dir/build: plainDrawer
.PHONY : CMakeFiles/plainDrawer.dir/build

CMakeFiles/plainDrawer.dir/requires: CMakeFiles/plainDrawer.dir/main.cpp.o.requires
CMakeFiles/plainDrawer.dir/requires: CMakeFiles/plainDrawer.dir/utils/Utilities.cpp.o.requires
CMakeFiles/plainDrawer.dir/requires: CMakeFiles/plainDrawer.dir/utils/Viewer.cpp.o.requires
.PHONY : CMakeFiles/plainDrawer.dir/requires

CMakeFiles/plainDrawer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plainDrawer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plainDrawer.dir/clean

CMakeFiles/plainDrawer.dir/depend:
	cd /home/felipe-narvaez/workspace/plainDrawer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/felipe-narvaez/workspace/plainDrawer/src /home/felipe-narvaez/workspace/plainDrawer/src /home/felipe-narvaez/workspace/plainDrawer/build /home/felipe-narvaez/workspace/plainDrawer/build /home/felipe-narvaez/workspace/plainDrawer/build/CMakeFiles/plainDrawer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plainDrawer.dir/depend

