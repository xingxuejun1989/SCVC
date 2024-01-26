# Specify the minimun camke version
CMAKE_MINIMUM_REQUIRED(VERSION 3.10 FATAL_ERROR)

# Create a project.
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
PROJECT(${PROJECT_NAME})

#Start support for the C++ standard
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
# The explicit requirement indicates that the C++ standard is supported.
SET(CMAKE_CXX_STANDARD 14)
# 
SET(CMAKE_C_STANDARD 14)
# The explicit requirement indicates that the C standard is supported,
SET(CMAKE_C_STANDARD_REQUIRED ON)


set(${PROJECT_NAME}_HEADERS

         src/algo/point_cloud_registration.h
         src/algo/registration/registration_helpers.hpp
         src/algo/registration/registration_SCVC.h
         src/algo/registration/registration_stdafx.h
         #src/algo/registration/registration_IcpOptimizer.h


         src/algo/ICP_Princeton/KDtree.h
         src/algo/ICP_Princeton/ICP.h
         src/algo/ICP_Princeton/TriMesh.h
         src/algo/ICP_Princeton/TriMesh_algo.h
        )
		


set(${PROJECT_NAME}_SOURCES
        
         src/algo/point_cloud_registration.cpp
         src/algo/registration/registration_helpers.cpp
         src/algo/registration/registration_SCVC.cpp
        # src/algo/registration/registration_IcpOptimizer.cpp

		
         src/algo/ICP_Princeton/KDtree.cpp
         src/algo/ICP_Princeton/ICP.cpp
         src/algo/ICP_Princeton/TriMesh_bounding.cpp
         src/algo/ICP_Princeton/TriMesh_connectivity.cpp
         src/algo/ICP_Princeton/TriMesh_curvature.cpp
         src/algo/ICP_Princeton/TriMesh_grid.cpp
         src/algo/ICP_Princeton/TriMesh_io.cpp
         src/algo/ICP_Princeton/TriMesh_normal.cpp
         src/algo/ICP_Princeton/TriMesh_pointareas.cpp
         src/algo/ICP_Princeton/TriMesh_stats.cpp
         src/algo/ICP_Princeton/TriMesh_tstrips.cpp
         src/algo/ICP_Princeton/remove.cpp
         src/algo/ICP_Princeton/reorder_verts.cpp
         src/main.cpp

        )


# Find third-party dependent libraries.
 FIND_PACKAGE(PCL 1.10.1 REQUIRED)

# Add an executable file.
ADD_EXECUTABLE(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

if (MSVC)
	add_definitions(/bigobj)
endif(MSVC)

# Add header file directory
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})


# Add library directory
TARGET_LINK_DIRECTORIES(${PROJECT_NAME} PRIVATE ${PCL_LIBRARY_DIRS})

# Add compiler definition
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

# Cancel the existing compiler definition.
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/UBOOST_ALL_NO_LIB-DBOOST_ALL_NO_LIB")

# 添加依赖库文件
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
