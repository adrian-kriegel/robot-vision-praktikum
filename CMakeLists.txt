
###############################################################################

MIRA_REQUIRE_PACKAGE(RVStudent)

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_LIST_DIR}/include
)

ADD_DEFINITIONS(-Wno-unused-function)

###############################################################################

MIRA_ADD_LIBRARY(ColorSegmentation
    SHARED
    SOURCE
        src/color-segmentation-rg.cpp
        #src/color-segmentation-rgb.cpp
		src/util.cpp
    LINK_LIBS
		StudentTemplate
    	Reference
)

MIRA_ADD_LIBRARY(OpticalFlow
    SHARED
    SOURCE
        src/OpticalFlow.C
    LINK_LIBS
		StudentTemplate
		Reference
)

###############################################################################

MIRA_INSTALL(
	DIRECTORY src
	DESTINATION domains/rvpractical/student
	PACKAGE RVStudent
	PATTERN ".svn" EXCLUDE
)

MIRA_INSTALL(
	FILES CMakeLists.txt RVStudent.package
	DESTINATION domains/rvpractical/student
	PACKAGE RVStudent
)