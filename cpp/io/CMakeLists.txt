# aptuner_io: event loop and async runtime, MAVLink codec, transports (TCP / UDP / serial), the MAVLink link,
# auto-connect, log download and system queries.
add_library(aptuner_io STATIC
  src/loop.cpp
  src/mavlink.cpp
  src/transport.cpp
  src/link.cpp
  src/sysinfo.cpp
  src/logdl.cpp
  src/autoconnect.cpp
)
target_include_directories(aptuner_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Threads REQUIRED)
target_link_libraries(aptuner_io PUBLIC aptuner_core Threads::Threads)
if(WIN32)
  target_link_libraries(aptuner_io PUBLIC ws2_32 setupapi)
endif()
