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




set(ONE_ROTATION_PROJECTION
	benchmark/one_rotation_projection/ProjectionProblem.cpp
	benchmark/one_rotation_projection/GeneratedExpressionSolver.cpp
	benchmark/one_rotation_projection/TypedExpressionSolverDepth10Eig.cpp
	benchmark/one_rotation_projection/CeresExpressionSolver.cpp
	benchmark/one_rotation_projection/HandTunedExpressionSolver.cpp
)

set(TWO_ROTATIONS_PROJECTION
	benchmark/two_rotations_projection/ProjectionProblem.cpp
	benchmark/two_rotations_projection/GeneratedExpressionSolver.cpp
	benchmark/two_rotations_projection/TypedExpressionSolverDepth10Eig.cpp
	benchmark/two_rotations_projection/CeresExpressionSolver.cpp
)
set(1_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.1.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.1.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.1.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.1.cpp
)
set(2_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.2.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.2.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.2.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.2.cpp
)
set(3_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.3.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.3.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.3.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.3.cpp
)
set(4_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.4.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.4.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.4.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.4.cpp
)
set(5_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.5.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.5.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.5.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.5.cpp
)
set(6_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.6.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.6.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.6.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.6.cpp
)
set(7_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.7.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.7.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.7.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.7.cpp
)
set(8_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.8.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.8.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.8.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.8.cpp
)
set(9_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.9.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.9.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.9.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.9.cpp
)
set(10_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.10.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.10.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.10.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.10.cpp
)
set(11_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.11.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.11.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.11.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.11.cpp
)
set(12_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.12.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.12.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.12.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.12.cpp
)
set(13_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.13.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.13.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.13.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.13.cpp
)
set(14_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.14.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.14.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.14.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.14.cpp
)
set(15_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.15.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.15.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.15.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.15.cpp
)
set(16_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.16.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.16.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.16.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.16.cpp
)
set(17_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.17.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.17.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.17.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.17.cpp
)
set(18_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.18.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.18.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.18.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.18.cpp
)
set(19_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.19.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.19.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.19.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.19.cpp
)
set(20_ROTATIONS
	benchmark/N_rotations/N/RotationProblem.cpp.20.cpp
	benchmark/N_rotations/N/HandTunedExpressionSolver.cpp.20.cpp
	benchmark/N_rotations/N/TypedExpressionSolverDepth10Eig.cpp.20.cpp
	benchmark/N_rotations/N/CeresExpressionSolver.cpp.20.cpp
)

set(N_ROTATIONS
	${1_ROTATIONS}
	${2_ROTATIONS}
	${3_ROTATIONS}
	${4_ROTATIONS}
	${5_ROTATIONS}
	${6_ROTATIONS}
	${7_ROTATIONS}
	${8_ROTATIONS}
	${9_ROTATIONS}
	${10_ROTATIONS}
	${11_ROTATIONS}
	${12_ROTATIONS}
	${13_ROTATIONS}
	${14_ROTATIONS}
	${15_ROTATIONS}
	${16_ROTATIONS}
	${17_ROTATIONS}
	${18_ROTATIONS}
	${19_ROTATIONS}
	${20_ROTATIONS}
)

rosbuild_add_executable(benchmarker 
	benchmark/malloc_count.c
	benchmark/StopWatch.cpp
	benchmark/EvalVariants.cpp
	benchmark/Benchmarker.cpp
#	${N_ROTATIONS}
#	${ONE_ROTATION_PROJECTION}
#	${TWO_ROTATIONS_PROJECTION}
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

rosbuild_add_executable(optimise_rotations_tex
    benchmark/StopWatch.cpp
    seals/optimise_rotations.cpp
    seals/rotation_errors.cpp
    seals/states.cpp
)
rosbuild_add_executable(optimise_rotations_ceres
	benchmark/StopWatch.cpp
    seals/optimise_rotations.cpp
    seals/rotation_errors.cpp
    seals/states.cpp
)
set_target_properties(optimise_rotations_ceres PROPERTIES COMPILE_FLAGS "-DUSE_CERES_AD=1")

rosbuild_add_executable(optimise_rotations_tex_m
    benchmark/StopWatch.cpp
    seals/optimise_rotations.cpp
    seals/rotation_errors.cpp
    seals/states.cpp
)
set_target_properties(optimise_rotations_tex_m PROPERTIES COMPILE_FLAGS "-DMEASURE_IN_REPEAT=1")

rosbuild_add_executable(optimise_rotations_ceres_m
	benchmark/StopWatch.cpp
    seals/optimise_rotations.cpp
    seals/rotation_errors.cpp
    seals/states.cpp
)
set_target_properties(optimise_rotations_ceres_m PROPERTIES COMPILE_FLAGS "-DUSE_CERES_AD=1 -DMEASURE_IN_REPEAT=1")

#target_link_libraries(optimise_rotations ${CERES_LIBRARIES} ${CGNUPLOT_LIBRARIES})

