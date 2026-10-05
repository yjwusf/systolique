cmake_minimum_required(VERSION 3.16...3.31)
project(systolique LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(SYSTOLIQUE_WERROR "Treat compiler warnings in the project's sources as errors" OFF)
option(SYSTOLIQUE_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SYSTOLIQUE_RTL "Build the optional RTL lockstep bench when Verilator and the Verilog exist" ON)

# Warnings for the project's own sources (not for Verilator's generated code).
set(SYSTOLIQUE_WARNINGS -Wall -Wextra)
if(SYSTOLIQUE_WERROR)
  list(APPEND SYSTOLIQUE_WARNINGS -Werror)
endif()
if(SYSTOLIQUE_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

enable_testing()
set(SYSTOLIQUE_REF_DIR ${PROJECT_SOURCE_DIR}/tests/reference/gemmini_rtl)
set(SYSTOLIQUE_CONFIGS default dim4 tiled ws_tree os16)

# ---------------------------------------------------------------- core library (standard C++ only)
add_library(systolique
  src/config.cpp
  src/types.cpp
  src/pe.cpp
  src/tile.cpp
  src/mesh.cpp
  src/transposer.cpp
  src/tag_queue.cpp
  src/mesh_with_delays.cpp
  src/vcd.cpp
  src/systolic_array.cpp
  src/frontend_config.cpp
  src/isa.cpp
  src/micro_ops.cpp
  src/scratchpad.cpp
  src/execute_controller.cpp
  src/reservation_station.cpp
  src/loop_matmul.cpp
  src/command_path.cpp
  src/controller.cpp
  src/host.cpp
  src/engine.cpp)
target_include_directories(systolique PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_options(systolique PRIVATE ${SYSTOLIQUE_WARNINGS})

# ---------------------------------------------------------------- bench library (traces need zlib)
find_package(ZLIB REQUIRED)
add_library(systolique_bench
  bench/ports.cpp
  bench/trace_io.cpp
  bench/stimulus.cpp
  bench/reference.cpp
  bench/run.cpp)
target_include_directories(systolique_bench PUBLIC ${PROJECT_SOURCE_DIR}/bench)
target_link_libraries(systolique_bench PUBLIC systolique ZLIB::ZLIB)
target_compile_options(systolique_bench PRIVATE ${SYSTOLIQUE_WARNINGS})

# ---------------------------------------------------------------- tests
function(systolique_test_program name)
  add_executable(${name} tests/${name}.cpp)
  target_link_libraries(${name} PRIVATE systolique_bench)
  target_compile_options(${name} PRIVATE ${SYSTOLIQUE_WARNINGS})
endfunction()

# Every stored RTL trace: replay (every output port, every cycle), regeneration from its
# stimulus, conservation, WS provenance, matmul results vs plain C++.
systolique_test_program(trace_test)
foreach(cfg ${SYSTOLIQUE_CONFIGS})
  add_test(NAME systolique_trace_${cfg}
           COMMAND trace_test --ref-dir ${SYSTOLIQUE_REF_DIR} --config ${cfg})
  set_tests_properties(systolique_trace_${cfg} PROPERTIES LABELS trace
    PASS_REGULAR_EXPRESSION "TRACES config=${cfg} .* mismatches=0 missing=0 conservation_failures=0 .*provenance_failures=0"
    FAIL_REGULAR_EXPRESSION "FAIL")
endforeach()

# Conservation of the accounting and provenance against values, every cycle.
systolique_test_program(conservation_test)
systolique_test_program(provenance_test)
foreach(cfg ${SYSTOLIQUE_CONFIGS})
  add_test(NAME systolique_conservation_${cfg} COMMAND conservation_test --config ${cfg})
  set_tests_properties(systolique_conservation_${cfg} PROPERTIES LABELS conservation
    PASS_REGULAR_EXPRESSION "CONSERVATION config=${cfg} runs=[1-9][0-9]* errors=0")
  add_test(NAME systolique_provenance_${cfg} COMMAND provenance_test --config ${cfg})
  set_tests_properties(systolique_provenance_${cfg} PROPERTIES LABELS provenance
    PASS_REGULAR_EXPRESSION "PROVENANCE config=${cfg} runs=[1-9][0-9]* checks=[1-9][0-9]* failures=0")
endforeach()

# The class rules and the accounting example of perf_reports/.
systolique_test_program(classes_test)
systolique_test_program(example_test)
add_test(NAME systolique_classes COMMAND classes_test)
set_tests_properties(systolique_classes PROPERTIES LABELS unit PASS_REGULAR_EXPRESSION "CLASSES ok")
add_test(NAME systolique_accounting_example COMMAND example_test)
set_tests_properties(systolique_accounting_example PROPERTIES LABELS unit
  PASS_REGULAR_EXPRESSION "EXAMPLE ok")

# The VCD writer changes no output and no count.
systolique_test_program(vcd_test)
add_test(NAME systolique_vcd COMMAND vcd_test --ref-dir ${SYSTOLIQUE_REF_DIR}
         --out ${CMAKE_CURRENT_BINARY_DIR}/vcd_out)
set_tests_properties(systolique_vcd PROPERTIES LABELS vcd PASS_REGULAR_EXPRESSION "VCD ok")

# ---------------------------------------------------------------- tools: dump + HTML viewer
add_executable(systolique_dump tools/array_dump.cpp)
target_link_libraries(systolique_dump PRIVATE systolique_bench)
target_compile_options(systolique_dump PRIVATE ${SYSTOLIQUE_WARNINGS})

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  # The viewer pages are well formed, offline and consistent; the committed examples are what
  # their commands generate.
  add_test(NAME systolique_view
           COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/array_view.py --selftest
                   --dump $<TARGET_FILE:systolique_dump> --out ${CMAKE_CURRENT_BINARY_DIR}/view_test)
  set_tests_properties(systolique_view PROPERTIES LABELS view PASS_REGULAR_EXPRESSION "VIEW_SELFTEST ok")
  # The stored traces are the files provenance.json lists (sha256, cycles), the configurations
  # agree with config.cpp, the RTL generator sources are the recorded ones.
  add_test(NAME systolique_reference_provenance
           COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/rtl_provenance.py check-traces)
  set_tests_properties(systolique_reference_provenance PROPERTIES LABELS trace
    PASS_REGULAR_EXPRESSION "TRACES ok")
else()
  message(WARNING "systolique: no Python 3; skipping the viewer and reference provenance tests")
endif()

# ---------------------------------------------------------------- optional: Gemmini RTL
if(SYSTOLIQUE_RTL)
  include(rtl/rtl.cmake)
endif()
