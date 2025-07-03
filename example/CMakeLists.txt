add_executable(demo  "demo.cc")

target_link_libraries(demo
  PRIVATE pure_pursuit 
  PRIVATE cubic_spline
)

set_target_properties(demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
