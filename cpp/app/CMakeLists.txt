# aptuner: the application (local HTTP + SSE server for the web UI, vehicle link, live tuner, guided flight,
# Telemetry + USB session, live notch). A window-subsystem program on Windows (no console window).
add_library(aptuner_app STATIC
  common.cpp
  liveaxis.cpp
  journal.cpp
  paramwrite.cpp
  tuner.cpp
  notchlive.cpp
  flight.cpp
  groundtune.cpp
  http.cpp
  server.cpp
)
target_include_directories(aptuner_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(aptuner_app PUBLIC aptuner_io)
if(WIN32)
  target_link_libraries(aptuner_app PUBLIC bcrypt)   # BCryptGenRandom: the per-run API token
endif()

add_executable(aptuner app_main.cpp)
target_link_libraries(aptuner PRIVATE aptuner_app)
if(WIN32)
  enable_language(RC)
  target_sources(aptuner PRIVATE aptuner.rc)
  set_target_properties(aptuner PROPERTIES WIN32_EXECUTABLE TRUE)
  target_link_libraries(aptuner PRIVATE shell32)
endif()
