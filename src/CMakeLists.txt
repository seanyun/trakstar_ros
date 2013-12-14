add_library(trakstar PointATC3DG.cpp)
add_dependencies(trakstar trakstar_gencpp)

target_link_libraries(trakstar
	#yaml-cpp
	${catkin_LIBRARIES}
	#${PC_LIBUSB_LIBRARIES}
	usb
)

install(TARGETS trakstar
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

