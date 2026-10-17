add_executable(aptuner-tests
  test_main.cpp
  test_build_info.cpp
)
target_link_libraries(aptuner-tests PRIVATE aptuner_core)

add_test(NAME unit COMMAND aptuner-tests)
# the programs start and report their version
add_test(NAME analyze_version COMMAND aptuner-analyze --version)
add_test(NAME app_version COMMAND aptuner --version)
add_test(NAME analyze_usage COMMAND aptuner-analyze)
set_tests_properties(analyze_usage PROPERTIES WILL_FAIL TRUE)   # no arguments: usage + exit code 2
