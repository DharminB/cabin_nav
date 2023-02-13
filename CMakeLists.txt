cmake_minimum_required(VERSION 3.10.2)
project(cabin_nav)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    tf
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
    roslib
    interactive_markers
    geometry_common
    yaml_common
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        tf
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        cv_bridge
        geometry_common
        yaml_common
        roslib
        interactive_markers
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ===============
# UTILITY LIBRARY
# ===============
add_library(utilities
    src/utils/utils.cpp
    src/structs/trajectory_point.cpp
)
target_link_libraries(utilities
    ${catkin_LIBRARIES}
)

add_library(mpc
    src/mpc/model.cpp
    src/mpc/optimiser.cpp
)
target_link_libraries(mpc
    ${catkin_LIBRARIES}
    utilities
)

add_library(initial_guess_utils
    src/utils/initial_guess_utils.cpp
)
target_link_libraries(initial_guess_utils
    ${catkin_LIBRARIES}
    utilities
)

add_library(aruco_marker_utils
    src/utils/aruco_marker_utils.cpp
)
target_link_libraries(aruco_marker_utils
    ${OpenCV_LIBRARIES}
)

add_library(geometric_planner
    src/utils/geometric_planner.cpp
    src/utils/lattice_search_utils.cpp
    src/structs/occupancy_grid.cpp
    src/structs/lattice.cpp
)
target_link_libraries(geometric_planner
    ${catkin_LIBRARIES}
    utilities
)

add_library(semantic_map
    src/semantic_map/semantic_map.cpp
    src/semantic_map/area.cpp
    src/semantic_map/connection.cpp
    src/semantic_map/topology_node.cpp
    src/semantic_map/topology_connection.cpp
)
target_link_libraries(semantic_map
    ${catkin_LIBRARIES}
    utilities
)

add_library(context_data
    src/structs/context_data.cpp
)
target_link_libraries(context_data
    ${catkin_LIBRARIES}
    utilities
    semantic_map
)

add_library(obstacle_tracker
    src/utils/obstacle_tracker.cpp
)
target_link_libraries(obstacle_tracker
    ${catkin_LIBRARIES}
    utilities
)

add_library(voronoi_calculator
    src/utils/voronoi_calculator.cpp
)
target_link_libraries(voronoi_calculator
    ${catkin_LIBRARIES}
    utilities
)

# ===================
# BEHAVIOR LIBRARIES
# ===================
macro(add_behavior_lib NAME LIBS)
    add_library(${NAME}_behavior
        src/behavior/${NAME}_behavior.cpp
    )
    target_link_libraries(${NAME}_behavior
        ${catkin_LIBRARIES}
        ${LIBS}
    )
endmacro()

add_library(behavior
    src/behavior/behavior.cpp
)
target_link_libraries(behavior
    ${catkin_LIBRARIES}
    mpc
    utilities
    context_data
)

add_behavior_lib("standstill" "behavior")
add_behavior_lib("joypad" "behavior")
add_behavior_lib("ptp" "behavior;obstacle_tracker;geometric_planner;initial_guess_utils")
add_behavior_lib("ptp_occ_grid" "ptp_behavior;voronoi_calculator")
add_behavior_lib("recovery_ptp" "ptp_behavior")
add_behavior_lib("corridor" "behavior;geometric_planner;initial_guess_utils")
add_behavior_lib("junction" "behavior;initial_guess_utils")
add_behavior_lib("door" "behavior")
add_behavior_lib("open_area" "behavior")

# ================
# ACTION LIBRARIES
# ================
macro(add_action_lib NAME LIBS)
    add_library(${NAME}_action
        src/action/${NAME}_action.cpp
        src/action/action.cpp
    )
    target_link_libraries(${NAME}_action
        ${catkin_LIBRARIES}
        ${LIBS}
    )
endmacro()

add_action_lib("goto" "behavior")

# ===============
# INPUT LIBRARIES
# ===============
macro(add_input_lib NAME LIBS)
    add_library(${NAME}_input
        src/input/${NAME}_input.cpp
        src/input/${NAME}_input_data.cpp
    )
    target_link_libraries(${NAME}_input
        ${catkin_LIBRARIES}
        ${LIBS}
    )
endmacro()

add_input_lib("velocity" "utilities")
add_input_lib("localisation" "utilities")
add_input_lib("joypad" "utilities")
add_input_lib("laser" "utilities")
add_input_lib("pointcloud" "utilities")
add_input_lib("image" "utilities")
add_input_lib("semantic_map" "utilities;semantic_map")

add_library(occupancy_grid_map_input
    src/input/occupancy_grid_map_input.cpp
    src/input/occupancy_grid_map_input_data.cpp
    src/structs/occupancy_grid.cpp
)
target_link_libraries(occupancy_grid_map_input
    ${catkin_LIBRARIES}
    utilities
)

# ===============
# OUTPUT LIBRARIES
# ===============
macro(add_output_lib NAME LIBS)
    add_library(${NAME}_output
        src/output/${NAME}_output.cpp
        src/output/${NAME}_output_data.cpp
    )
    target_link_libraries(${NAME}_output
        ${catkin_LIBRARIES}
        ${LIBS}
    )
endmacro()

add_output_lib("cmd_vel" "utilities")
add_output_lib("visualization_marker" "utilities")

# =====================
# CABIN NAVIGATOR LIBRARY
# =====================
add_library(cabin_navigator
    SHARED
        src/cabin_navigator.cpp
        src/task/task_manager.cpp
        src/task/task_planner.cpp
        src/behavior/behavior_factory.cpp
        src/action/action_factory.cpp
)
target_link_libraries(cabin_navigator
    ${catkin_LIBRARIES}
    yaml-cpp
    utilities
    semantic_map
    # actions
    goto_action
    # behaviors
    behavior
    standstill_behavior
    joypad_behavior
    ptp_behavior
    ptp_occ_grid_behavior
    corridor_behavior
    junction_behavior
    door_behavior
    open_area_behavior
    recovery_ptp_behavior
    # inputs
    velocity_input
    localisation_input
    joypad_input
    laser_input
    pointcloud_input
    image_input
    occupancy_grid_map_input
    semantic_map_input
    # outputs
    cmd_vel_output
    visualization_marker_output
)

# =====================
# INPUT MANAGER LIBRARY
# =====================
add_library(input_manager
    src/input/input_manager.cpp
    src/input/input_factory.cpp
)
target_link_libraries(input_manager
    ${catkin_LIBRARIES}
    utilities
    velocity_input
    localisation_input
    joypad_input
    laser_input
    pointcloud_input
    image_input
    occupancy_grid_map_input
    semantic_map_input
)

# ======================
# OUTPUT MANAGER LIBRARY
# ======================
add_library(output_manager
    src/output/output_manager.cpp
    src/output/output_factory.cpp
)
target_link_libraries(output_manager
    ${catkin_LIBRARIES}
    utilities
    cmd_vel_output
    visualization_marker_output
)

# ==========
# EXECUTABLE
# ==========
add_executable(cabin_navigator_ros
    src/cabin_navigator_ros.cpp
    src/main.cpp
)
target_link_libraries(cabin_navigator_ros
    ${catkin_LIBRARIES}
    cabin_navigator
    input_manager
    output_manager
)

add_executable(semantic_map_editor
    src/utils/semantic_map_editor.cpp
)
target_link_libraries(semantic_map_editor
    ${catkin_LIBRARIES}
    utilities
    semantic_map
)

# =====
# TESTS
# =====
if (CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif ()

# =======
# INSTALL
# =======
install(TARGETS cabin_navigator cabin_navigator_ros semantic_map_editor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
