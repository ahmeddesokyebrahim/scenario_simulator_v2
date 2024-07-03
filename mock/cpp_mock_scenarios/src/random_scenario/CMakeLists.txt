generate_parameter_library(
  common_parameters
  random_parameters/common.yaml
)

ament_auto_add_executable(random001
  random001.cpp
)
target_link_libraries(random001
  cpp_scenario_node
  common_parameters
)
ament_auto_add_executable(random002
  scenario_base.cpp
)
target_link_libraries(random002
  cpp_scenario_node
  common_parameters
)
ament_auto_add_executable(random003
  scenario_base.cpp
)
target_link_libraries(random003
  cpp_scenario_node
  common_parameters
)
target_include_directories(random001
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/common_parameters/include
)

install(TARGETS
  random001
  random002
  random003
  DESTINATION lib/cpp_mock_scenarios
)

install(
  DIRECTORY random_parameters
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  # add_cpp_mock_scenario_test(${PROJECT_NAME} "random001" "10.0")
endif()
