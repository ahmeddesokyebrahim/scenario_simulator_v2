generate_parameter_library(
  random001_parameters
  random_parameters/random001.yaml
)

ament_auto_add_executable(random001
  # random001.cpp
  random002-odaiba.cpp
)
target_link_libraries(random001
  cpp_scenario_node
  random001_parameters
)
target_include_directories(random001
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
)

install(TARGETS
  random001
  DESTINATION lib/cpp_mock_scenarios
)

ament_auto_add_executable(npc-not-stopping-issue
        npc-not-stopping-issue.cpp
        )
target_link_libraries(npc-not-stopping-issue
        cpp_scenario_node
        random001_parameters
        )
target_include_directories(npc-not-stopping-issue
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
        )

install(TARGETS
        npc-not-stopping-issue
        DESTINATION lib/cpp_mock_scenarios
        )



ament_auto_add_executable(npc-not-stopping-issue-simple
        npc-not-stopping-issue-simple.cpp
        )
target_link_libraries(npc-not-stopping-issue-simple
        cpp_scenario_node
        random001_parameters
        )
target_include_directories(npc-not-stopping-issue-simple
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
        )

install(TARGETS
        npc-not-stopping-issue-simple
        DESTINATION lib/cpp_mock_scenarios
        )




ament_auto_add_executable(npc-not-stopping-issue-automovement
        npc-not-stopping-issue-automovement.cpp
        )
target_link_libraries(npc-not-stopping-issue-automovement
        cpp_scenario_node
        random001_parameters
        )
target_include_directories(npc-not-stopping-issue-automovement
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/random001_parameters/include
        )

install(TARGETS
        npc-not-stopping-issue-automovement
        DESTINATION lib/cpp_mock_scenarios
        )

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  # add_cpp_mock_scenario_test(${PROJECT_NAME} "random001" "10.0")
endif()
