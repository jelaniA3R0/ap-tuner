add_executable(aptuner-tests
  test_main.cpp
  test_build_info.cpp
  test_golden_core.cpp
  test_cascade.cpp
  test_heavy.cpp
  test_dfreader.cpp
  test_logintegrity.cpp
  test_reallogs.cpp
  test_io.cpp
  test_liveaxis.cpp
  test_live_safety.cpp
  test_write_verify.cpp
  test_server_gate.cpp
)
target_link_libraries(aptuner-tests PRIVATE aptuner_app)
target_compile_definitions(aptuner-tests PRIVATE APTUNER_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden")

add_test(NAME unit COMMAND aptuner-tests -live_ -wv_ -heavylog_)
# model tuning on the tandem QuadPlane logs (skipped when they are not on this PC; about a minute)
add_test(NAME heavy_logs COMMAND aptuner-tests heavylog_)
# the in-flight safety scenarios on a fake vehicle (about a minute: real-time watches)
add_test(NAME live_safety COMMAND aptuner-tests live_)
# verified writes bound to the right aircraft (audit R3) on fake vehicles (real-time: about two minutes)
add_test(NAME write_verify COMMAND aptuner-tests wv_)
# the programs start and report their version
add_test(NAME analyze_version COMMAND aptuner-analyze --version)
add_test(NAME app_version COMMAND aptuner --version)
add_test(NAME analyze_usage COMMAND aptuner-analyze)
set_tests_properties(analyze_usage PROPERTIES WILL_FAIL TRUE)   # no arguments: usage + exit code 2
# a file that is not a DataFlash log: refused with exit code 3 and the reason
add_test(NAME analyze_refuses_non_log
         COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:aptuner-analyze> -DWORK=${CMAKE_CURRENT_BINARY_DIR}
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/cli_refusal.cmake)
