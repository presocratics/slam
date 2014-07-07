# -------------------------------------------------------------------- #
# Author: Martin Miller
#         miller7@illinois.edu
# -------------------------------------------------------------------- #

cmake_minimum_required (VERSION 2.6)
project (slam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# set build directory
set(CMAKE_CURRENT_BINARY_DIR ${PROJECT_SOURCE_DIR}/build)

	MESSAGE(STATUS "--------------------------------------------------")
    MESSAGE(STATUS "Project: ARC Slam ")
	MESSAGE(STATUS "by Martin Miller <miller7@illinois.edu>")
	MESSAGE(STATUS "--------------------------------------------------")

# -------------------------------------------------------------------- #
# Change to configure for your system / preferences
# -------------------------------------------------------------------- #

    SET( CMAKE_BUILD_TYPE Debug ) 							# set to [Release|Debug] to switch optimization flags
	find_package(OpenCV REQUIRED)

# -------------------------------------------------------------------- #
# Application setup
# -------------------------------------------------------------------- #

	SET ( APPLICATION_NAME slam )
    FILE( GLOB_RECURSE PROJECT_SOURCES src/slam/*.cpp )

# -------------------------------------------------------------------- #
# Change with caution!
# -------------------------------------------------------------------- #

	MESSAGE(STATUS "Building -- ${CMAKE_BUILD_TYPE} -- Version")
	
	SET(CMAKE_VERBOSE_MAKEFILE ON)
	SET(OPTIMIZATION_FLAG_DEBUG "-O0 -pg")
	SET(OPTIMIZATION_FLAG_RELEASE "-O3 -march=native -mtune=native -msse3 -mfpmath=sse -fomit-frame-pointer -finline-functions -floop-optimize -ffinite-math-only")
    SET(WARNING_FLAG_DEBUG "-g -Werror -Wall -Winit-self -Wsequence-point -Wunused")
    SET(WARNING_FLAG_RELEASE "-Winit-self -Wsequence-point -Wunused -Wno-deprecated")

	IF( CMAKE_BUILD_TYPE MATCHES Release )
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPTIMIZATION_FLAG_RELEASE} -fmessage-length=0 -pipe ${WARNING_FLAG_RELEASE}")
	ELSE( CMAKE_BUILD_TYPE MATCHES Debug )
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPTIMIZATION_FLAG_DEBUG} -fmessage-length=0 -pipe ${WARNING_FLAG_DEBUG}")
	ENDIF( CMAKE_BUILD_TYPE MATCHES Release )

# -------------------------------------------------------------------- #
# Setting up Library linkages
# -------------------------------------------------------------------- #

	INCLUDE_DIRECTORIES(
		${OpenCV_INCLUDE_DIRS}
		${PROJECT_SOURCE_DIR}/include
	)

	ADD_EXECUTABLE (
		${APPLICATION_NAME}
		${PROJECT_SOURCES}
	)

	TARGET_LINK_LIBRARIES (
		${APPLICATION_NAME}
		${OpenCV_LIBS}
	)
