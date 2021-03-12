INCLUDE_DIRECTORIES(${CAMSENSE_SOURCE_DIR}/src/base)
LINK_DIRECTORIES(${CAMSENSE_SOURCE_DIR}/src/base)

ADD_SUBDIRECTORY(base)

ADD_EXECUTABLE (sdk_demo sdk_demo.cpp)
TARGET_LINK_LIBRARIES (sdk_demo lidar)

ADD_EXECUTABLE (sdk_V3_demo sdk_V3_demo.cpp LidarTest.cpp)
TARGET_LINK_LIBRARIES (sdk_V3_demo lidar)

ADD_EXECUTABLE (sdk_scanData sdk_scanData.cpp)
TARGET_LINK_LIBRARIES (sdk_scanData lidar)
