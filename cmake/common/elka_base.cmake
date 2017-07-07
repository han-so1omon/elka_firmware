############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include(CMakeParseArguments)

#=============================================================================
#
#	elka_parse_function_args
#
#	This function simplifies usage of the cmake_parse_arguments module.
#	It is intended to be called by other functions.
#
#	Usage:
#		elka_parse_function_args(
#			NAME <name>
#			[ OPTIONS <list> ]
#			[ ONE_VALUE <list> ]
#			[ MULTI_VALUE <list> ]
#			REQUIRED <list>
#			ARGN <ARGN>)
#
#	Input:
#		NAME		: the name of the calling function
#		OPTIONS		: boolean flags
#		ONE_VALUE	: single value variables
#		MULTI_VALUE	: multi value variables
#		REQUIRED	: required arguments
#		ARGN		: the function input arguments, typically ${ARGN}
#
#	Output:
#		The function arguments corresponding to the following are set:
#		${OPTIONS}, ${ONE_VALUE}, ${MULTI_VALUE}
#
#	Example:
#		function test()
#			elka_parse_function_args(
#				NAME TEST
#				ONE_VALUE NAME
#				MULTI_VALUE LIST
#				REQUIRED NAME LIST
#				ARGN ${ARGN})
#			message(STATUS "name: ${NAME}")
#			message(STATUS "list: ${LIST}")
#		endfunction()
#
#		test(NAME "hello" LIST a b c)
#
#		OUTPUT:
#			name: hello
#			list: a b c
#
function(elka_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			if (NOT "${OUT_${arg}}" STREQUAL "0")
				message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
			endif()
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

#=============================================================================
#
#	elka_add_git_submodule
#
#	This function add a git submodule target.
#
#	Usage:
#		elka_add_git_submodule(TARGET <target> PATH <path>)
#
#	Input:
#		PATH		: git submodule path
#
#	Output:
#		TARGET		: git target
#
#	Example:
#		elka_add_git_submodule(TARGET git_nuttx PATH "NuttX")
#
function(elka_add_git_submodule)
	elka_parse_function_args(
		NAME elka_add_git_submodule
		ONE_VALUE TARGET PATH
		REQUIRED TARGET PATH
		ARGN ${ARGN})
	string(REPLACE "/" "_" NAME ${PATH})
  add_custom_command(OUTPUT ${ELKA_BINARY_DIR}/git_init_${NAME}.stamp
    WORKING_DIRECTORY ${ELKA_SOURCE_DIR}
    COMMAND touch ${ELKA_BINARY_DIR}/git_init_${NAME}.stamp
    DEPENDS ${ELKA_SOURCE_DIR}/.gitmodules
		)
	add_custom_target(${TARGET}
    WORKING_DIRECTORY ${ELKA_SOURCE_DIR}
# todo:Not have 2 list of submodules one (see the end of Tools/check_submodules.sh and Firmware/CMakeLists.txt)
# using the list of submodules from the CMake file to drive the test
#		COMMAND Tools/check_submodules.sh ${PATH}
    DEPENDS ${ELKA_BINARY_DIR}/git_init_${NAME}.stamp
		)
endfunction()



#=============================================================================
#
#	elka_generate_messages
#
#	This function generates source code from ROS msg definitions.
#
#	Usage:
#		elka_generate_messages(TARGET <target> MSGS <msg-files>)
#
#	Input:
#		MSG_FILES	: the ROS msgs to generate files from
#		OS			: the operating system selected
#		DEPENDS		: dependencies
#
#	Output:
#		TARGET		: the message generation target
#
#	Example:
#		elka_generate_messages(TARGET <target>
#			MSG_FILES <files> OS <operating-system>
#			[ DEPENDS <dependencies> ]
#			)
#
function(elka_generate_messages)
	elka_parse_function_args(
		NAME elka_generate_messages
		OPTIONS VERBOSE
		ONE_VALUE OS TARGET
		MULTI_VALUE MSG_FILES DEPENDS INCLUDES
		REQUIRED MSG_FILES OS TARGET
		ARGN ${ARGN})
	set(QUIET)
	if(NOT VERBOSE)
		set(QUIET "-q")
  endif()

	# headers
  set(msg_out_path ${ELKA_BINARY_DIR}/inc/uORB/topics)
	set(msg_list)
	foreach(msg_file ${MSG_FILES})
		get_filename_component(msg ${msg_file} NAME_WE)
		list(APPEND msg_list ${msg})
	endforeach()
	set(msg_files_out)
	foreach(msg ${msg_list})
		list(APPEND msg_files_out ${msg_out_path}/${msg}.h)
	endforeach()
	add_custom_command(OUTPUT ${msg_files_out}
		COMMAND ${PYTHON_EXECUTABLE}
			tools/px_generate_uorb_topic_files.py
			--headers
			${QUIET}
			-f ${MSG_FILES}
			-i ${INCLUDES}
			-o ${msg_out_path}
			-e msg/templates/uorb
      -t ${ELKA_BINARY_DIR}/topics_temporary_header
		DEPENDS ${DEPENDS} ${MSG_FILES}
    WORKING_DIRECTORY ${ELKA_SOURCE_DIR}
		COMMENT "Generating uORB topic headers"
		VERBATIM
		)

	# !sources
  set(msg_source_out_path	${ELKA_BINARY_DIR}/topics_sources)
	set(msg_source_files_out ${msg_source_out_path}/uORBTopics.cpp)
	foreach(msg ${msg_list})
		list(APPEND msg_source_files_out ${msg_source_out_path}/${msg}.cpp)
	endforeach()
	add_custom_command(OUTPUT ${msg_source_files_out}
		COMMAND ${PYTHON_EXECUTABLE}
			tools/px_generate_uorb_topic_files.py
			--sources
			${QUIET}
			-f ${MSG_FILES}
			-i ${INCLUDES}
			-o ${msg_source_out_path}
			-e msg/templates/uorb
      -t ${ELKA_BINARY_DIR}/topics_temporary_sources
		DEPENDS ${DEPENDS} ${MSG_FILES}
    WORKING_DIRECTORY ${ELKA_SOURCE_DIR}
		COMMENT "Generating uORB topic sources"
		VERBATIM
		)
	set_source_files_properties(${msg_source_files_out} PROPERTIES GENERATED TRUE)

	# We remove uORBTopics.cpp to make sure the generator is re-run, which is
	# necessary when a .msg file is removed and because uORBTopics.cpp depends
	# on all topics.
	execute_process(COMMAND rm uORBTopics.cpp
		WORKING_DIRECTORY ${msg_source_out_path}
		ERROR_QUIET)

	# multi messages for target OS
	set(msg_multi_out_path
    ${ELKA_BINARY_DIR}/src/platforms/${OS}/elka_messages)
	set(msg_multi_files_out)
	foreach(msg ${msg_list})
		list(APPEND msg_multi_files_out ${msg_multi_out_path}/elka_${msg}.h)
	endforeach()
	add_custom_command(OUTPUT ${msg_multi_files_out}
		COMMAND ${PYTHON_EXECUTABLE}
			tools/px_generate_uorb_topic_files.py
			--headers
			${QUIET}
			-f ${MSG_FILES}
			-i ${INCLUDES}
			-o ${msg_multi_out_path}
			-e msg/templates/px4/uorb
      -t ${ELKA_BINARY_DIR}/multi_topics_temporary/${OS}
			-p "elka_"
		DEPENDS ${DEPENDS} ${MSG_FILES}
    WORKING_DIRECTORY ${ELKA_SOURCE_DIR}
		COMMENT "Generating uORB topic multi headers for ${OS}"
		VERBATIM
		)

	elka_add_library(${TARGET}
		${msg_source_files_out}
		${msg_multi_files_out}
		${msg_files_out}
		)
endfunction()

#=============================================================================
#
#	elka_add_optimization_flags_for_target
#
set(all_posix_cmake_targets "" CACHE INTERNAL "All cmake targets for which optimization can be suppressed")
function(elka_add_optimization_flags_for_target target)
	set(_no_optimization_for_target FALSE)
	# If the current CONFIG is posix_sitl_* then suppress optimization for certain targets.
	if(CONFIG MATCHES "^posix_sitl_")
    foreach(_regexp $ENV{ELKA_NO_OPTIMIZATION})
			if("${target}" MATCHES "${_regexp}")
				set(_no_optimization_for_target TRUE)
				set(_matched_regexp "${_regexp}")
			endif()
		endforeach()
		# Create a full list of targets that optimization can be suppressed for.
		list(APPEND all_posix_cmake_targets ${target})
		set(all_posix_cmake_targets ${all_posix_cmake_targets} CACHE INTERNAL "All cmake targets for which optimization can be suppressed")
	endif()
	if(NOT ${_no_optimization_for_target})
		target_compile_options(${target} PRIVATE ${optimization_flags})
	else()
		message(STATUS "Disabling optimization for target '${target}' because it matches the regexp '${_matched_regexp}' in env var ELKA_NO_OPTIMIZATION")
		target_compile_options(${target} PRIVATE -O0)
	endif()
	# Pass variable to the parent px4_add_library.
	set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	elka_add_library
#
#	Like add_library but with optimization flag fixup.
#
function(elka_add_library target)
	add_library(${target} ${ARGN})
	elka_add_optimization_flags_for_target(${target})
	# Pass variable to the parent px4_add_module.
	set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)

  set(ELKA_LINK_LIBS
    "${ELKA_LINK_LIBS} ${target}"
  )
endfunction()

# vim: set noet fenc=utf-8 ff=unix nowrap:
