# This file contains backwards compatibility patches for various legacy functions and variables
# Functions

function(PROTOBUF_GENERATE_CPP SRCS HDRS)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
    return()
  endif()

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
    set(_append_arg APPEND_PATH)
  endif()

  if(DEFINED Protobuf_IMPORT_DIRS)
    set(_import_arg IMPORT_DIRS ${Protobuf_IMPORT_DIRS})
  endif()

  set(_outvar)
  protobuf_generate(${append_arg} LANGUAGE cpp OUT_VAR _outvar ${_import_arg} PROTOS ${ARGN})

  set(${SRCS})
  set(${HDRS})
  foreach(_file ${_outvar})
    if(_file MATCHES "cc$")
      list(APPEND ${SRCS} ${_file})
    else()
      list(APPEND ${HDRS} ${_file})
    endif()
  endforeach()
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

function(PROTOBUF_GENERATE_PYTHON SRCS)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_PYTHON() called without any proto files")
    return()
  endif()

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
    set(_append_arg APPEND_PATH)
  endif()

  if(DEFINED Protobuf_IMPORT_DIRS)
    set(_import_arg IMPORT_DIRS ${Protobuf_IMPORT_DIRS})
  endif()

  set(_outvar)
  protobuf_generate(${append_arg} LANGUAGE cpp OUT_VAR _outvar ${_import_arg} PROTOS ${ARGN})
  set(${SRCS} ${_outvar} PARENT_SCOPE)
endfunction()

# Environment

# Backwards compatibility
# Define camel case versions of input variables
foreach(UPPER
    PROTOBUF_SRC_ROOT_FOLDER
    PROTOBUF_IMPORT_DIRS
    PROTOBUF_DEBUG
    PROTOBUF_LIBRARY
    PROTOBUF_PROTOC_LIBRARY
    PROTOBUF_INCLUDE_DIR
    PROTOBUF_PROTOC_EXECUTABLE
    PROTOBUF_LIBRARY_DEBUG
    PROTOBUF_PROTOC_LIBRARY_DEBUG
    PROTOBUF_LITE_LIBRARY
    PROTOBUF_LITE_LIBRARY_DEBUG
    )
    if (DEFINED ${UPPER})
        string(REPLACE "PROTOBUF_" "Protobuf_" Camel ${UPPER})
        if (NOT DEFINED ${Camel})
            set(${Camel} ${${UPPER}})
        endif()
    endif()
endforeach()

if(DEFINED Protobuf_SRC_ROOT_FOLDER)
  message(AUTHOR_WARNING "Variable Protobuf_SRC_ROOT_FOLDER defined, but not"
    " used in CONFIG mode")
endif()

include(SelectLibraryConfigurations)

# Internal function: search for normal library as well as a debug one
#    if the debug one is specified also include debug/optimized keywords
#    in *_LIBRARIES variable
function(_protobuf_find_libraries name filename)
  if(${name}_LIBRARIES)
    # Use result recorded by a previous call.
  elseif(${name}_LIBRARY)
    # Honor cache entry used by CMake 3.5 and lower.
    set(${name}_LIBRARIES "${${name}_LIBRARY}" PARENT_SCOPE)
  else()
    get_target_property(${name}_LIBRARY_RELEASE protobuf::lib${filename}
      LOCATION_RELEASE)
    get_target_property(${name}_LIBRARY_DEBUG protobuf::lib${filename}
      LOCATION_DEBUG)

    select_library_configurations(${name})
    set(${name}_LIBRARY ${${name}_LIBRARY} PARENT_SCOPE)
    set(${name}_LIBRARIES ${${name}_LIBRARIES} PARENT_SCOPE)
  endif()
endfunction()

# Internal function: find threads library
function(_protobuf_find_threads)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads)
    if(Threads_FOUND)
        list(APPEND PROTOBUF_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
        set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBRARIES}" PARENT_SCOPE)
    endif()
endfunction()

#
# Main.
#

# By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
  set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
endif()

# The Protobuf library
_protobuf_find_libraries(Protobuf protobuf)

# The Protobuf Lite library
_protobuf_find_libraries(Protobuf_LITE protobuf-lite)

# The Protobuf Protoc Library
_protobuf_find_libraries(Protobuf_PROTOC protoc)

if(UNIX)
  _protobuf_find_threads()
endif()

# Set the include directory
get_target_property(Protobuf_INCLUDE_DIRS protobuf::libprotobuf
  INTERFACE_INCLUDE_DIRECTORIES)

# Set the protoc Executable
get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
  IMPORTED_LOCATION_RELEASE)
if(NOT EXISTS "${Protobuf_PROTOC_EXECUTABLE}")
  get_target_property(Protobuf_PROTOC_EXECUTABLE protobuf::protoc
    IMPORTED_LOCATION_DEBUG)
endif()

# Version info variable
set(Protobuf_VERSION "3.4.1")

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Protobuf
    REQUIRED_VARS Protobuf_PROTOC_EXECUTABLE Protobuf_LIBRARIES Protobuf_INCLUDE_DIRS
    VERSION_VAR Protobuf_VERSION
)

# Backwards compatibility
# Define upper case versions of output variables
foreach(Camel
    Protobuf_VERSION
    Protobuf_SRC_ROOT_FOLDER
    Protobuf_IMPORT_DIRS
    Protobuf_DEBUG
    Protobuf_INCLUDE_DIRS
    Protobuf_LIBRARIES
    Protobuf_PROTOC_LIBRARIES
    Protobuf_LITE_LIBRARIES
    Protobuf_LIBRARY
    Protobuf_PROTOC_LIBRARY
    Protobuf_INCLUDE_DIR
    Protobuf_PROTOC_EXECUTABLE
    Protobuf_LIBRARY_DEBUG
    Protobuf_PROTOC_LIBRARY_DEBUG
    Protobuf_LITE_LIBRARY
    Protobuf_LITE_LIBRARY_DEBUG
    )
    string(TOUPPER ${Camel} UPPER)
    set(${UPPER} ${${Camel}})
endforeach()
