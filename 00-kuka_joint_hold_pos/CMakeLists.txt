# create an executable
SET(GCC_COVERAGE_COMPILE_FLAGS "-g3")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEMO_APPLICATION_BINARY_DIR}/00-kuka_joint_hold_pos)
ADD_EXECUTABLE (controller00 controller.cpp)
ADD_EXECUTABLE (controller_pos_ang controller_operational_pos_ang.cpp)
ADD_EXECUTABLE (controller_pos_ang_in_nullspace controller_pos_ang_in_nullspace.cpp)
ADD_EXECUTABLE (controller_operational_pos controller_operational_pos.cpp)
ADD_EXECUTABLE (controller_joint controller_joint.cpp)
ADD_EXECUTABLE (controller_operations_Keven controller_operations_Keven.cpp)
ADD_EXECUTABLE (simviz00 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller00 ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_pos_ang ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_pos_ang_in_nullspace ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_operational_pos ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_joint ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_operations_Keven ${DEMO_APPLICATION_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz00 ${DEMO_APPLICATION_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${DEMO_APPLICATION_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/00-kuka_joint_hold_pos)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf iiwa7.urdf DESTINATION ${APP_RESOURCE_DIR})