
cmake_minimum_required(VERSION 3.4)

find_package(LLVM REQUIRED CONFIG)

MESSAGE( "include dirs" ) 
MESSAGE( ${LLVM_INCLUDE_DIRS} ) 
MESSAGE( "lib dirs" ) 
MESSAGE( ${LLVM_LIBRARY_DIRS} ) 

add_definitions(${LLVM_DEFINITIONS})

include_directories("~/llvm_final/build/include")
include_directories("~/llvm_final/llvm/include")
include_directories("~/llvm_final/build/tools/clang/include")
include_directories("~/llvm_final/llvm/tools/clang/include")

link_directories(${LLVM_LIBRARY_DIRS})

add_subdirectory(skeleton)  # Use your pass name here.
