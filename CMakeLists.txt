# Specify the minimun camke version
CMAKE_MINIMUM_REQUIRED(VERSION 3.1 FATAL_ERROR)

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
   
         src/algo/ICP_Princeton/KDtree.h
         src/algo/ICP_Princeton/ICP.h
         src/algo/ICP_Princeton/TriMesh.h
         src/algo/ICP_Princeton/TriMesh_algo.h

        )
		


set(${PROJECT_NAME}_SOURCES
        
         src/algo/point_cloud_registration.cpp
         src/algo/registration/registration_helpers.cpp
         src/algo/registration/registration_SCVC.cpp
  
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
 #FIND_PACKAGE(PCL 1.10.1 REQUIRED)

### found OpenMP libraries
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

#find_package (Eigen3 REQUIRED NO_MODULE)






if (MSVC)
    add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
	add_compile_definitions(/bigobj)
endif(MSVC)


### Configuration
set(3DMODEL_ROOT         ${CMAKE_CURRENT_LIST_DIR})
set(3DMODEL_THIRD_PARTY  ${3DMODEL_ROOT}/3rd_party)

add_subdirectory(3rdParty)
mark_as_advanced(FORCE BUILD_SHARED_LIBS)
mark_as_advanced(FORCE BUILD_TESTING)




# Add an executable file.
ADD_EXECUTABLE(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE ${3DMODEL_THIRD_PARTY})

set(EIGEN_SOURCE_DIR ${3DMODEL_THIRD_PARTY}/eigen-3.4.0/)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN_SOURCE_DIR})


