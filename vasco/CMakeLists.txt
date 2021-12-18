set(LLVM_LINK_COMPONENTS core support irreader passes)
include_directories(./include)
add_compile_definitions(debug_level=100)
add_library(Driver SHARED
	# include/Analysis.h
	src/Analysis.cpp
	# src/UserAnalysis.cpp
	# src/UserAnalysisBackwardOnly.cpp
	# src/UserAnalysisForwardOnly.cpp
	# src/DataFlowValue.cpp
	# src/UserDataFlowValueForward.cpp
	# src/UserDataFlowValueBackward.cpp
)
set_target_properties(Driver PROPERTIES
	COMPILE_FLAGS "-std=c++14 -fno-rtti -g"
)
llvm_config (Driver ${LLVM_LINK_COMPONENTS})
#target_link_libraries(Driver PRIVATE /usr/local/lib/libbprinter.a)
include(GNUInstallDirs)
# install(TARGETS LLVM_VASCO_ABSTRACTION LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES
./include/Analysis.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
