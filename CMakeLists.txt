project(ISO_object LANGUAGES C CXX)
cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--disable-new-dtags")

# Manage namespace of the project
set(ISOOBJECT_LIBRARY ${PARENT_TARGET_NAMESPACE}${PROJECT_NAME})
set(ISOOBJECT_TEST ${ISOOBJECT_LIBRARY}Demo)
set(ISOOBJECT_EMBEDDED ${ISOOBJECT_LIBRARY}Embedded)

#ISO22133 library/headers
add_subdirectory(iso22133)
set(ISO22133_LIBRARY ISO22133)


# Set the RPATH to the install directory
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/")
# Tell linker to set RPATH to the install directory, outside the build tree
# Note: this has to be done before add_executable/add_library.
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include(GNUInstallDirs)
find_package(Boost REQUIRED COMPONENTS system)

# Library target
add_library(${ISOOBJECT_LIBRARY} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133object.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/iso22133state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/trajDecoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bikeObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/embeddedIsoObject.cpp
)

target_include_directories(${ISOOBJECT_LIBRARY} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot
    ${CMAKE_CURRENT_SOURCE_DIR}/iso22133
)
set(ISO_OBJ_LIBS ${ISO22133_LIBRARY} atomic pthread Boost::system)
target_link_libraries(${ISOOBJECT_LIBRARY} ${ISO_OBJ_LIBS})

set_property(TARGET ${ISOOBJECT_LIBRARY} PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/iso22133object.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/iso22133state.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/trajDecoder.hpp
)

# Try to remove this
set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/bikeObject.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot/signal.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/iso22133/iso22133.h
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/tcpServer.hpp
)

set_property(TARGET ${ISOOBJECT_LIBRARY} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/udpServer.hpp
)

find_package(Boost REQUIRED COMPONENTS program_options)

# Demo target
add_executable(${ISOOBJECT_TEST} demoIsoObject.cpp)
target_include_directories(${ISOOBJECT_TEST} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(${ISOOBJECT_TEST} LINK_PUBLIC
    ${ISOOBJECT_LIBRARY}
    Boost::program_options
)

# The shared library for embedded use
add_library(${ISOOBJECT_EMBEDDED} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/embeddedIsoObject.cpp
)
set_property(TARGET ${ISOOBJECT_EMBEDDED} APPEND PROPERTY
    PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/inc/embeddedIsoObject.hpp
)
target_include_directories(${ISOOBJECT_EMBEDDED} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(${ISOOBJECT_EMBEDDED} LINK_PUBLIC
    ${ISOOBJECT_LIBRARY}
    Boost::program_options
)

install(CODE "MESSAGE(STATUS \"Installing target ${ISOOBJECT_LIBRARY}\")")
install(TARGETS ${ISOOBJECT_LIBRARY}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
