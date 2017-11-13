include_directories(${MicroMaestro_SOURCE_DIR}/include ${PSYNC_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp MicroMaestro.cpp Servo.cpp RPMSerialInterface.cpp RPMSerialInterfacePOSIX.cpp)
target_link_libraries(${PROJECT_NAME} ${PSYNC_LIBS})
