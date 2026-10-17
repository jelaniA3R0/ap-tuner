add_executable(aptuner-tests
  test_main.cpp
  test_build_info.cpp
  test_golden_core.cpp
  test_dfreader.cpp
  test_io.cpp
  test_liveaxis.cpp
  test_live_safety.cpp
  test_server_gate.cpp
)
target_link_libraries(aptuner-tests PRIVATE aptuner_app)
target_compile_definitions(aptuner-tests PRIVATE APTUNER_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden")

add_test(NAME unit COMMAND aptuner-tests -live_)
# the in-flight safety scenarios on a fake vehicle (about a minute: real-time watches)
add_test(NAME live_safety COMMAND aptuner-tests live_)
# the programs start and report their version
add_test(NAME analyze_version COMMAND aptuner-analyze --version)
add_test(NAME app_version COMMAND aptuner --version)
add_test(NAME analyze_usage COMMAND aptuner-analyze)
set_tests_properties(analyze_usage PROPERTIES WILL_FAIL TRUE)   # no arguments: usage + exit code 2
