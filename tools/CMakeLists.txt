add_executable(print PointATC3DG_test.cpp)
add_dependencies(print trakstar)
target_link_libraries(print trakstar ${catkin_LIBRARIES})

install(TARGETS print
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
