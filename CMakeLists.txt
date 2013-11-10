cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

add_definitions(-std=c++11  -Wno-enum-compare -Wno-unused-local-typedefs -gdwarf-2)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
# eigen 2 or 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/local/include/eigen2
	/usr/local/include/eigen
	/usr/include/eigen3
	/usr/include/eigen2
	/usr/include/eigen
	/opt/ros/diamondback/stacks/geometry/eigen/include
)
include_directories(${EIGEN_INCLUDE_DIR})

add_executable(test_pe src/test_pe.cpp)

add_executable(test_pge_one benchmark/one_rotation_projection/test_pge.cpp)
add_executable(test_pge_tow benchmark/two_rotations_projection/test_pge.cpp)

add_executable(TestTypedExpresssions src/TestTypedExpressions.cpp)
#set_target_properties(TestTypedExpresssions PROPERTIES COMPILE_FLAGS "-fverbose-asm -Wa,-adhln=TestTypedExpresssions.S,")



rosbuild_add_executable(benchmarker 
	benchmark/malloc_count.c
	benchmark/StopWatch.cpp
	benchmark/EvalVariants.cpp
	benchmark/Benchmarker.cpp

	benchmark/one_rotation_projection/ProjectionProblem.cpp
	benchmark/one_rotation_projection/GeneratedExpressionSolver.cpp
	benchmark/one_rotation_projection/TypedExpressionSolverDepth10Eig.cpp
	benchmark/one_rotation_projection/CeresExpressionSolver.cpp
	benchmark/one_rotation_projection/HandTunedExpressionSolver.cpp

	benchmark/two_rotations_projection/ProjectionProblem.cpp
	benchmark/two_rotations_projection/GeneratedExpressionSolver.cpp
	benchmark/two_rotations_projection/TypedExpressionSolverDepth10Eig.cpp
	benchmark/two_rotations_projection/CeresExpressionSolver.cpp
)
target_link_libraries(benchmarker dl rt)
#set_target_properties(benchmarker PROPERTIES COMPILE_FLAGS "-finline-limit=10000000")
target_link_libraries(benchmarker boost_program_options)


#add_executable(test_exp src/test.cpp 
#     src/Expressions.cpp
#     src/ExpressionImpl.cpp)
     
rosbuild_add_gtest(test_requirements
  test/requirements.cpp 
  test/TestTools.cpp
  test/test_main.cpp
)

rosbuild_add_gtest(test_expressions
  test/TestTools.cpp
  test/TestRotations.cpp
  test/TestQuaternions.cpp
  test/TestEuclideanPointsAndScalars.cpp
  test/test_main.cpp
)


# rosbuild_add_executable(prepare_eigen_problem src/prepare_eigen_problem.cpp)
# target_link_libraries(normalized_cut lapack blas)

#FIND_PACKAGE(Ceres REQUIRED)
#INCLUDE_DIRECTORIES(${CERES_INCLUDES} )

# set(CGNUPLOT $ENV{HOME}/sources/gtl/vsv_stack/cgnuplot)
# INCLUDE_DIRECTORIES(${CGNUPLOT}/include)
# LINK_DIRECTORIES(${CGNUPLOT}/lib)
# SET(CGNUPLOT_LIBRARIES cgnuplot)

rosbuild_add_executable(optimise_rotations
	benchmark/StopWatch.cpp
    seals/optimise_rotations.cpp
    seals/rotation_errors.cpp
    seals/states.cpp
)
#target_link_libraries(optimise_rotations ${CERES_LIBRARIES} ${CGNUPLOT_LIBRARIES})

