cmake_minimum_required(VERSION 2.6)

# define module name & interface version
set (module ar0144)

# define interface version
set (${module}_INTERFACE_CURRENT  1)
set (${module}_INTERFACE_REVISION 0)
set (${module}_INTERFACE_AGE      0)

# we want to compile all .c files as default
file(GLOB libsources source/AR0144.c )

# set public headers, these get installed
file(GLOB pub_headers include/*.h)

# define include paths
include_directories(
    include
    include_priv
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include
    )

# module specific defines
###add_definitions(-Wno-error=unused-function)

# add lib to build env
#add_library(${module}_static STATIC ${libsources})
add_library(${module}_shared SHARED ${libsources})

#SET_TARGET_PROPERTIES(${module}_static PROPERTIES OUTPUT_NAME     ${module})
#SET_TARGET_PROPERTIES(${module}_static PROPERTIES LINK_FLAGS      -static)
#SET_TARGET_PROPERTIES(${module}_static PROPERTIES FRAMEWORK       TRUE PUBLIC_HEADER "${pub_headers}")

SET_TARGET_PROPERTIES(${module}_shared PROPERTIES OUTPUT_NAME     ${module})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES LINK_FLAGS      -shared)
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES SOVERSION       ${${module}_INTERFACE_CURRENT})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES VERSION         ${${module}_INTERFACE_CURRENT}.${${module}_INTERFACE_REVISION}.${${module}_INTERFACE_AGE})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES FRAMEWORK       TRUE PUBLIC_HEADER "${pub_headers}")

# add convenience target: put sensor driver into the 'bin' output dir as well
if ( NOT ANDROID )
add_custom_target(${module}.drv
                  ALL
                  COMMAND ${CMAKE_COMMAND} -E copy ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib/lib${module}.so.${${module}_INTERFACE_CURRENT} ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/bin/${module}.drv
                  DEPENDS ${module}_shared
                  COMMENT "Copying ${module} driver module"
                  )
endif()

if(${APPMODE} STREQUAL NATIVE)
add_custom_target(${module}.init
                  ALL
		  DEPENDS ${module}_shared
                  COMMENT "Copying ${module} init files"
                  )
endif (${APPMODE} STREQUAL NATIVE)

# define lib dependencies
#target_link_libraries(${module}_static
#                      ${platform_libs}
#                      ${base_libs}
#                      ${drv_libs}
#                      isi_shared
#                      )

#target_link_libraries(${module}_shared
#                      ${platform_libs}
#                      ${base_libs}
#                      ${drv_libs}
#                      isi_shared
#                      )

# define stuff to install
#install(TARGETS ${module}_static
#        PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${module}
#        ARCHIVE         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#        )

install(TARGETS ${module}_shared
        PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${module}
        ARCHIVE         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${module}
        LIBRARY         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${module}
        )

# install the sensor driver as well, but to 'bin' location!
install(FILES       ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib/lib${module}.so.${${module}_INTERFACE_CURRENT}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        RENAME      ${module}.drv
        )

if( DEFINED APPSHELL_TOP_COMPILE)
add_custom_target(copy_shell_libs_${module} ALL
       COMMENT "##Copy libs to shell libs"
       COMMAND ${CMAKE_COMMAND} -E copy ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/lib/lib${module}.so ${CMAKE_HOME_DIRECTORY}/shell_libs/${APPMODE}/ispcore/${PLATFORM}/lib${module}.so
       #COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include/${module} ${CMAKE_HOME_DIRECTORY}/shell_libs/include/units_headers/${module}
)
add_dependencies(copy_shell_libs_${module} ${module}_shared)
endif( DEFINED APPSHELL_TOP_COMPILE)

# create common targets for this module
include(${UNITS_TOP_DIRECTORY}/targets.cmake)

# create calib data targets
add_subdirectory(calib)
