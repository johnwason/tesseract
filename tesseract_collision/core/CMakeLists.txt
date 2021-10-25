# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}_core
  INTERFACE Eigen3::Eigen
            tesseract::tesseract_common
            tesseract::tesseract_geometry
            Boost::boost
            Boost::system)
target_compile_options(${PROJECT_NAME}_core INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Create test suite interface
add_library(${PROJECT_NAME}_test_suite INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}_test_suite
  INTERFACE Eigen3::Eigen
            tesseract::tesseract_support
            tesseract::tesseract_geometry
            ${PROJECT_NAME}_core)
target_compile_options(${PROJECT_NAME}_test_suite INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_test_suite INTERFACE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_test_suite ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_suite INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_test_suite
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_test_suite INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")

# Create test suite benchmarks interface
add_library(${PROJECT_NAME}_test_suite_benchmarks INTERFACE)
target_link_libraries(${PROJECT_NAME}_test_suite_benchmarks INTERFACE Eigen3::Eigen tesseract::tesseract_support
                                                                      tesseract::tesseract_geometry)
target_compile_options(${PROJECT_NAME}_test_suite_benchmarks INTERFACE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_test_suite_benchmarks ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_test_suite_benchmarks INTERFACE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_test_suite_benchmarks
  INTERFACE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_test_suite_benchmarks INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

install_targets(
  TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_test_suite
  ${PROJECT_NAME}_test_suite_benchmarks)