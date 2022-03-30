set(LLVM_LINK_COMPONENTS core support irreader passes)
include_directories(./include)
include_directories(../TransformIRPass)
include_directories("/usr/local/include/spatial")
add_compile_definitions(debug_level=100)
add_library(Driver SHARED
	src/Analysis.cpp
        include/Context.h)
set_target_properties(Driver PROPERTIES
	COMPILE_FLAGS "-std=c++14 -fno-rtti -g -D Memory"
)

target_link_libraries(Driver PUBLIC Spatial)

llvm_config (Driver ${LLVM_LINK_COMPONENTS})
include(GNUInstallDirs)

install(TARGETS Driver LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})