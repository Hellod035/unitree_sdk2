find_library(MAIlBOX_LIB mailbox PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFAULT_PATH)
if (NOT MAIlBOX_LIB)
        message(FATAL_ERROR "mailbox library for the architecture is not found")
else ()
        message(STATUS "mailbox library found at: ${MAIlBOX_LIB}")
endif ()

add_library(mailbox STATIC IMPORTED GLOBAL)
target_include_directories(mailbox INTERFACE include)
set_target_properties(mailbox PROPERTIES
        IMPORTED_LOCATION ${MAIlBOX_LIB})