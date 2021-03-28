cmake_minimum_required(VERSION 3.9)
project(mylib VERSION 1.0.1 DESCRIPTION "mylib description")
include(GNUInstallDirs)
add_library(mylib SHARED src/mylib.c)


set_target_properties(mylib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER api/mylib.h)

configure_file(mylib.pc.in mylib.pc @ONLY)

target_include_directories(mylib PRIVATE .)

install(TARGETS mylib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${CMAKE_BINARY_DIR}/mylib.pc
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
