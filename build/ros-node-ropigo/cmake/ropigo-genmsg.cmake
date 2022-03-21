# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ropigo: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ropigo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv" NAME_WE)
add_custom_target(_ropigo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ropigo" "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ropigo
  "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ropigo
)

### Generating Module File
_generate_module_cpp(ropigo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ropigo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ropigo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ropigo_generate_messages ropigo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv" NAME_WE)
add_dependencies(ropigo_generate_messages_cpp _ropigo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ropigo_gencpp)
add_dependencies(ropigo_gencpp ropigo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ropigo_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ropigo
  "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ropigo
)

### Generating Module File
_generate_module_lisp(ropigo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ropigo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ropigo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ropigo_generate_messages ropigo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv" NAME_WE)
add_dependencies(ropigo_generate_messages_lisp _ropigo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ropigo_genlisp)
add_dependencies(ropigo_genlisp ropigo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ropigo_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ropigo
  "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ropigo
)

### Generating Module File
_generate_module_py(ropigo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ropigo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ropigo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ropigo_generate_messages ropigo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/ros-node-ropigo/srv/SimpleWrite.srv" NAME_WE)
add_dependencies(ropigo_generate_messages_py _ropigo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ropigo_genpy)
add_dependencies(ropigo_genpy ropigo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ropigo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ropigo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ropigo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ropigo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ropigo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ropigo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ropigo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ropigo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
