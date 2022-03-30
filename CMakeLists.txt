cmake_minimum_required(VERSION 3.16)
project(vasco)
set(LLVM_ROOT "/usr/lib/llvm-12" CACHE PATH "/usr/lib/llvm-12")

list(APPEND CMAKE_PREFIX_PATH "${LLVM_ROOT}/share/llvm/cmake")
find_package(LLVM REQUIRED 12.0.0)
list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
include(HandleLLVMOptions)
include(AddLLVM)

add_definitions(${LLVM_DEFINITIONS})
link_directories(${LLVM_LIBRARY_DIRS})
include_directories(${LLVM_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library(Spatial SHARED IMPORTED)
set_target_properties(Spatial PROPERTIES IMPORTED_LOCATION "/usr/local/lib/libSpatial.so")

add_subdirectory(vasco)

install(DIRECTORY
    vasco/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
install(DIRECTORY
TransformIRPass/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

