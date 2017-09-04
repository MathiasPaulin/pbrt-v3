cmake_minimum_required ( VERSION 2.8 )

project ( pbrt-kmeans )

# note remove libpbrt.a
add_library( kmgen src/kmgen.cpp src/kmedoids.cpp )

target_link_libraries(kmgen PRIVATE ../../../cmake-build-release/libpbrt.a)
target_include_directories (kmgen PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

