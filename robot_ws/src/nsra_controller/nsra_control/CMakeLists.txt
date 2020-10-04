include_directories(
  include/
  config/
)

# NSRA Hardware Interface
add_library(nsra_hw_interface
  src/nsra_hw_interface.cpp
)
target_link_libraries(nsra_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(nsra_hw_main src/nsra_hw_main.cpp)
target_link_libraries(nsra_hw_main
  nsra_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    nsra_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    nsra_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

# Install header files
install(DIRECTORY include/nsra_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
