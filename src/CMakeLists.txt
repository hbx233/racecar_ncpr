add_library(ncpr SHARED 
		trajectory_tracker.cpp)
target_link_libraries(ncpr ${catkin_LIBRARIES})