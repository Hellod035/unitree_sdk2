find_library(ULOG_CPP_LIB ulog_cpp PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFAULT_PATH)
if (NOT ULOG_CPP_LIB)
        message(FATAL_ERROR "ulog_cpp library for the architecture is not found")
else ()
        message(STATUS "ulog_cpp library found at: ${ULOG_CPP_LIB}")
endif ()

add_library(ulog_cpp STATIC IMPORTED GLOBAL)
target_include_directories(ulog_cpp INTERFACE include)
set_target_properties(ulog_cpp PROPERTIES
        IMPORTED_LOCATION ${ULOG_CPP_LIB})