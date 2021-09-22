# Add source files
aux_source_directory(algorithm SOURCE)
aux_source_directory(axis SOURCE)
aux_source_directory(fb SOURCE)
aux_source_directory(servo SOURCE)
aux_source_directory(tool SOURCE)

# Create "RTmotion" library
add_library(${PROJECT_NAME} SHARED ${SOURCE})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-variable)
if(PLOT)
  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PYTHON_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} ruckig::ruckig)
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE ruckig::ruckig)
endif(PLOT)

# Create single axis demo executable
add_executable(single_axis_move_relative demo/RT_single_axis_move_relative.cpp)
target_link_libraries(single_axis_move_relative ${PROJECT_NAME} ruckig::ruckig -lpthread)

add_executable(axis_move_cyclic demo/single_axis_move_relative.cpp)
target_link_libraries(axis_move_cyclic ${PROJECT_NAME} ruckig::ruckig -lpthread)

find_library(
    tcc_collector
    NAMES tcc_collector
    HINTS /usr/lib64/)
find_library(
    TCC_rtcdatamgr
    NAMES tcc_rtcdatamgr
    HINTS /usr/lib64/)

find_library(
    TCC_tcc
    NAMES tcc
    HINTS /usr/lib64/)

add_executable(tcc_measure demo/tcc_measure.cpp)
target_link_libraries(tcc_measure ${PROJECT_NAME} ${tcc_collector}  ${TCC_rtcdatamgr} ${TCC_tcc} ruckig::ruckig -lpthread)

add_executable(multi-axis demo/multi-axis.cpp)
target_link_libraries(multi-axis ${PROJECT_NAME} ruckig::ruckig -lpthread)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  TARGETS single_axis_move_relative axis_move_cyclic
  DESTINATION ${INSTALL_BINDIR}
)
