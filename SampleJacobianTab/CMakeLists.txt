cmake_minimum_required (VERSION 2.6)
project (Tutorial0Tab)

message (STATUS "Creating Makefile for Tutorial0Tab project")

if(WIN32)
set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
    message (STATUS "Setup Visual Studio Specific Flags")
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
set (CMAKE_DEBUG_POSTFIX "d")
set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
   set (CMAKE_INSTALL_PREFIX /usr )
   set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS -std=c++0x -fpermissive")
endif()

include (${CMAKE_INSTALL_PREFIX}/share/dart/DARTConfig.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

LINK_DIRECTORIES( ${DART_LIBRARY_DIRS} )
INCLUDE_DIRECTORIES( ${DART_INCLUDE_DIRS} )

# Search all header and source files
set ( tutorial_srcs  TutorialTabApp.cpp TutorialTab.cpp )
set ( tutorial_hdrs  TutorialTab.h  )
set ( JTFollower_srcs JTFollower/JTFollower.cpp )
set ( JTFollower_hdrs JTFollower/JTFollower.h )


if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)
find_package (wxWidgets)
if (wxWidgets_FOUND)

  include (${wxWidgets_USE_FILE})
  include_directories(${DART_INCLUDE_DIRS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

 if(MSVC)
  add_executable (TutorialTab WIN32 ${tutorial_srcs} ${tutorial_hdrs} 
    ${JTFollower_srcs}  ${JTFollower_hdrs} )
  set_target_properties (TutorialTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
else()
  add_executable (TutorialTab ${tutorial_srcs} ${tutorial_hdrs}
     ${JTFollower_srcs}  ${JTFollower_hdrs} )
endif()

  target_link_libraries (TutorialTab ${GRIP_LIBRARIES}  ${DART_LIBRARIES}  ${wxWidgets_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY} ${GLU_LIBRARIES} )

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
