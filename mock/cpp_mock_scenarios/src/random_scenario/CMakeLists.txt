generate_parameter_library(
  random001_parameters
  random_parameters/random001.yaml
)

#random001
ament_auto_add_executable(random001
  random001.cpp
)
target_link_libraries(random001
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random001
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)

#random002
ament_auto_add_executable(random002-odaiba
  random002-odaiba.cpp
)
target_link_libraries(random002-odaiba
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random002-odaiba
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)

#random003
ament_auto_add_executable(random003
  random003-bs.cpp
)
target_link_libraries(random003
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random003
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)

#random002 - test
  ament_auto_add_executable(random001-npc-not-stopping-issue
  random001-npc-not-stopping-issue.cpp
)
target_link_libraries(random001-npc-not-stopping-issue
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random001-npc-not-stopping-issue
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)

#random002 - test simplifiest
ament_auto_add_executable(random001-npc-not-stopping-issue-simple
random001-npc-not-stopping-issue-simple.cpp
)
target_link_libraries(random001-npc-not-stopping-issue-simple
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random001-npc-not-stopping-issue-simple
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)


install(TARGETS
  random001
  random002-odaiba
  random003
  random001-npc-not-stopping-issue
  random001-npc-not-stopping-issue-simple
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  # add_cpp_mock_scenario_test(${PROJECT_NAME} "random001" "10.0")
endif()
