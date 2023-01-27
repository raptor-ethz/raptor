cmake_minimum_required(VERSION 3.1.0)


set( C_FILES
ftconfig.c
ftrt.c
dom.c
expatls.c
node.c
stack.c
xmlparse.c
xmlrole.c
xmltok.c)

add_library(ATIDAQ ${HEADER_FILES} ${C_FILES}) 


# message(STATUS "HEADER_FILES : ${HEADER_FILES}")
# message(STATUS "C_FILES : ${C_FILES}")

# message(STATUS "IMPL : ${IMPL}")
# message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")


target_include_directories(ATIDAQ PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})