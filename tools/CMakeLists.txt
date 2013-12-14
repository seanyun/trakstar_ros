add_executable(trakstar_print PointATC3DG_test.cpp)
add_dependencies(trakstar_print trakstar)
target_link_libraries(trakstar_print trakstar ${catkin_LIBRARIES})

install(TARGETS trakstar_print
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
