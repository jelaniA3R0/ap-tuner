add_executable(aptuner-analyze analyze_main.cpp)
target_link_libraries(aptuner-analyze PRIVATE aptuner_core)

add_executable(aptuner-link link_main.cpp)
target_link_libraries(aptuner-link PRIVATE aptuner_io)

add_executable(aptuner-heavy heavy_main.cpp)
target_link_libraries(aptuner-heavy PRIVATE aptuner_core)
