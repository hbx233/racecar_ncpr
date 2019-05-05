add_executable(trajectory_test trajectory_test.cpp)

add_executable(poly_test poly_test.cpp)
target_link_libraries(poly_test ${catkin_LIBRARIES})

add_executable(tracker_test tracker_test.cpp)
target_link_libraries(tracker_test ncpr)

add_executable(global_planner_test global_planner_test.cpp)
target_link_libraries(global_planner_test ${catkin_LIBRARIES})

add_executable(local_planner_test local_planner_test.cpp)
target_link_libraries(local_planner_test ncpr)