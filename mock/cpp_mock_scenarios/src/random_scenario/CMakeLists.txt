generate_parameter_library(
  random001_parameters
  random_parameters/random001.yaml
)

ament_auto_add_executable(random001
  random001.cpp
)
target_link_libraries(random001
  cpp_scenario_node
  random001_parameters
)
ament_auto_add_executable(random002
  random002-miraikan.cpp
)
target_link_libraries(random002
  cpp_scenario_node
  random001_parameters
)
ament_auto_add_executable(random003
  random003-bs.cpp
)
target_link_libraries(random003
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random001
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)

install(TARGETS
  random001
  random002
  random003
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  # add_cpp_mock_scenario_test(${PROJECT_NAME} "random001" "10.0")
endif()
