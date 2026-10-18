# aptuner-analyze refuses a file that is not a DataFlash log: exit code 3 and the reason, as tools/analyze-log.js.
#   cmake -DEXE=<aptuner-analyze> -DWORK=<folder> -P cli_refusal.cmake
set(f "${WORK}/aptuner-refusal-empty.bin")
file(WRITE "${f}" "")
execute_process(COMMAND "${EXE}" "${f}" RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
file(REMOVE "${f}")
if(NOT rc EQUAL 3)
  message(FATAL_ERROR "exit code ${rc}, expected 3:\n${out}${err}")
endif()
if(NOT out MATCHES "NOT ANALYSED \\(empty\\): The file is empty\\.")
  message(FATAL_ERROR "unexpected output:\n${out}")
endif()
message(STATUS "refused with exit code 3: ${out}")
