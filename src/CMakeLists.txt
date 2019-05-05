add_library(ncpr SHARED 
		trajectory_tracker.cpp
		local_planner.cpp)
target_link_libraries(ncpr ${catkin_LIBRARIES})