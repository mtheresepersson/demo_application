# create an executable
SET(GCC_COVERAGE_COMPILE_FLAGS "-g3")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEMO_APPLICATION_BINARY_DIR}/01-kuka-mobile-base)
ADD_EXECUTABLE (controller01 controller.cpp)
ADD_EXECUTABLE (controller_joint_grasp controller_joint_grasp.cpp)
ADD_EXECUTABLE (simviz01 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller01 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_joint_grasp ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz01 ${DEMO_APPLICATION_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${DEMO_APPLICATION_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/01-kuka-mobile-base)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf iiwa7.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY robotObject.urdf DESTINATION ${APP_RESOURCE_DIR})