file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)


add_executable(map_sender ${SOURCES})
add_dependencies(map_sender map_sender_gencpp)
target_link_libraries(map_sender ${catkin_LIBRARIES})

install(TARGETS map_sender
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)