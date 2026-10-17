# aptuner_core: the math and analysis library (no I/O). Ported module by module from ../src (JavaScript);
# every module gets golden tests against the JavaScript results (see ../tests).
add_library(aptuner_core STATIC
  src/build_info.cpp
  src/cx.cpp
  src/fft.cpp
  src/spectra.cpp
  src/stepresp.cpp
  src/controller.cpp
  src/plant.cpp
  src/synth.cpp
  src/rules.cpp
  src/timeid.cpp
  src/jsnum.cpp
  src/json.cpp
  src/log/dfreader.cpp
  src/log/integrity.cpp
  src/engine/vehicle.cpp
  src/engine/analysis.cpp
  src/engine/loganalysis.cpp
  src/engine/notch.cpp
  src/engine/issues.cpp
  src/engine/report.cpp
)
target_include_directories(aptuner_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/generated
)
target_link_libraries(aptuner_core PUBLIC aptuner_options)
target_compile_features(aptuner_core PUBLIC cxx_std_20)

# shortest round-trip double formatting: std::to_chars when the standard library has it for double
# (libc++ provides it without defining __cpp_lib_to_chars), otherwise a printf-based fallback
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "${CMAKE_CXX20_STANDARD_COMPILE_OPTION}")
check_cxx_source_compiles("
#include <charconv>
int main() { char b[64]; auto r = std::to_chars(b, b + 64, 0.1, std::chars_format::scientific); return r.ptr == b; }
" APTUNER_HAVE_FLOAT_TO_CHARS)
if(APTUNER_HAVE_FLOAT_TO_CHARS)
  target_compile_definitions(aptuner_core PRIVATE APTUNER_HAVE_FLOAT_TO_CHARS=1)
endif()
