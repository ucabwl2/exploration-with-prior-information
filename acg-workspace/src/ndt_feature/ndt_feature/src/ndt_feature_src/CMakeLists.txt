add_library(ndt_feature_lib ndt_feature_fuser_hmt.cpp ndt_feature_graph.cpp ndt_feature_link.cpp motion_model.cpp)
add_dependencies(ndt_feature_lib ${catkin_EXPORTED_TARGETS})
