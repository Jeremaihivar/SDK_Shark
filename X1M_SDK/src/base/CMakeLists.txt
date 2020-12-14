INCLUDE_DIRECTORIES(.)

SET(LIDAR_SRC lidar.cpp rOc_serial.cpp TimeOut.cpp)
ADD_LIBRARY(lidar ${LIDAR_SRC})
IF (WIN32)
ELSE()
    TARGET_LINK_LIBRARIES(lidar pthread)
ENDIF()