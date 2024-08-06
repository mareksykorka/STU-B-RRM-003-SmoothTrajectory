cmake_minimum_required(VERSION 3.0.2)
project(zad3)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_conversions
    moveit_ros_planning_interface
)

catkin_package()

include_directories(
    include
    include/zad3
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_joint_node
    # Node main code
    src/joint_trajectory/joint_trajectory.cpp
    # Custom Math
    src/custom_math/polynomial_tools.cpp
    src/custom_math/inverse_matrix_solver.cpp
    # Helper Code - logging, organization
    src/trajectory/trajectory_plan.cpp
    src/logger/logger.cpp
)

add_executable(${PROJECT_NAME}_cartesian_node
    # Node main code
    src/cartesian_trajectory/cartesian_trajectory.cpp
    src/ik_solver/ik_solver.cpp
    # Custom Math
    src/custom_math/polynomial_tools.cpp
    src/custom_math/inverse_matrix_solver.cpp
    # Helper Code - logging, organization
    src/trajectory/trajectory_plan.cpp
    src/logger/logger.cpp
)

add_dependencies(${PROJECT_NAME}_joint_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_cartesian_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_joint_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_cartesian_node ${catkin_LIBRARIES})