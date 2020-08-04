# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "localizer_dwm1001: 2 messages, 5 services")

set(MSG_I_FLAGS "-Ilocalizer_dwm1001:/home/fire/catkin_ws/src/dwm1001_ros/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(localizer_dwm1001_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" ""
)

get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" ""
)

get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" ""
)

get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" ""
)

get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" ""
)

get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" ""
)

get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" NAME_WE)
add_custom_target(_localizer_dwm1001_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "localizer_dwm1001" "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)
_generate_msg_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)

### Generating Services
_generate_srv_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_cpp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
)

### Generating Module File
_generate_module_cpp(localizer_dwm1001
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(localizer_dwm1001_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(localizer_dwm1001_generate_messages localizer_dwm1001_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_cpp _localizer_dwm1001_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localizer_dwm1001_gencpp)
add_dependencies(localizer_dwm1001_gencpp localizer_dwm1001_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localizer_dwm1001_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)
_generate_msg_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)

### Generating Services
_generate_srv_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_eus(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
)

### Generating Module File
_generate_module_eus(localizer_dwm1001
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(localizer_dwm1001_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(localizer_dwm1001_generate_messages localizer_dwm1001_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_eus _localizer_dwm1001_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localizer_dwm1001_geneus)
add_dependencies(localizer_dwm1001_geneus localizer_dwm1001_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localizer_dwm1001_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)
_generate_msg_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)

### Generating Services
_generate_srv_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_lisp(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
)

### Generating Module File
_generate_module_lisp(localizer_dwm1001
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(localizer_dwm1001_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(localizer_dwm1001_generate_messages localizer_dwm1001_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_lisp _localizer_dwm1001_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localizer_dwm1001_genlisp)
add_dependencies(localizer_dwm1001_genlisp localizer_dwm1001_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localizer_dwm1001_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)
_generate_msg_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)

### Generating Services
_generate_srv_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_nodejs(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
)

### Generating Module File
_generate_module_nodejs(localizer_dwm1001
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(localizer_dwm1001_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(localizer_dwm1001_generate_messages localizer_dwm1001_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_nodejs _localizer_dwm1001_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localizer_dwm1001_gennodejs)
add_dependencies(localizer_dwm1001_gennodejs localizer_dwm1001_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localizer_dwm1001_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)
_generate_msg_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)

### Generating Services
_generate_srv_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)
_generate_srv_py(localizer_dwm1001
  "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
)

### Generating Module File
_generate_module_py(localizer_dwm1001
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(localizer_dwm1001_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(localizer_dwm1001_generate_messages localizer_dwm1001_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Tag.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_1.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_0.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Tag_srv.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_3.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/msg/Anchor.msg" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fire/catkin_ws/src/dwm1001_ros/srv/Anchor_2.srv" NAME_WE)
add_dependencies(localizer_dwm1001_generate_messages_py _localizer_dwm1001_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(localizer_dwm1001_genpy)
add_dependencies(localizer_dwm1001_genpy localizer_dwm1001_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS localizer_dwm1001_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/localizer_dwm1001
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(localizer_dwm1001_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/localizer_dwm1001
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(localizer_dwm1001_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/localizer_dwm1001
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(localizer_dwm1001_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/localizer_dwm1001
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(localizer_dwm1001_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/localizer_dwm1001
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(localizer_dwm1001_generate_messages_py std_msgs_generate_messages_py)
endif()
