add_executable(trakstar_node trakstar_node.cpp)
add_dependencies(trakstar_node trakstar)

target_link_libraries(trakstar_node
	trakstar
	${catkin_LIBRARIES}
)

install(TARGETS trakstar_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

