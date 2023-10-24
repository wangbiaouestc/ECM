# library
set( LIB_NAME boost )

# create upper case name
#string( TOUPPER ${LIB_NAME} LIB_NAME_UC )

# get source files
#file( GLOB SRC_FILES "*.cpp" )

# get include files
#file( GLOB INC_FILES "*.hpp" )
#file( GLOB PUBLIC_INC_FILES  "../../../include/${LIB_NAME}/*.h" )
#set( INC_FILES ${PRIVATE_INC_FILES} ${PUBLIC_INC_FILES} )

# set resource file for MSVC compilers
#if( MSVC )
#  set( RESOURCE_FILE ${LIB_NAME}.rc )
#endif()

# library
add_library( ${LIB_NAME} INTERFACE )
#target_compile_definitions( ${LIB_NAME} PRIVATE ${LIB_NAME_UC}_SOURCE )
#target_compile_definitions( ${LIB_NAME} PUBLIC $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${LIB_NAME},TYPE>,SHARED_LIBRARY>>:${LIB_NAME_UC}_DYN_LINK> )
target_include_directories( ${LIB_NAME} INTERFACE . )
#target_link_libraries( ${LIB_NAME} Threads::Threads )

# example: place header files in different folders
#source_group( "Header Files"          FILES ${PUBLIC_INC_FILES} )
#source_group( "Header Files\\private" FILES ${PRIVATE_INC_FILES} )
#source_group( "Resource Files"        FILES ${RESOURCE_FILE} )

# set the folder where to place the projects
#set_target_properties( ${LIB_NAME} PROPERTIES FOLDER lib )
