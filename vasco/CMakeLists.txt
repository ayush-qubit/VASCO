set(LLVM_LINK_COMPONENTS core support irreader passes)
include_directories(./include)
add_compile_definitions(debug_level=100)
add_library(Driver SHARED
	src/Analysis.cpp
)
set_target_properties(Driver PROPERTIES
	COMPILE_FLAGS "-std=c++14 -fno-rtti -g"
)
llvm_config (Driver ${LLVM_LINK_COMPONENTS})
#target_link_libraries(Driver PRIVATE /usr/local/lib/libbprinter.a)
include(GNUInstallDirs)
