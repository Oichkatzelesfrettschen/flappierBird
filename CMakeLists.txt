cmake_minimum_required(VERSION 3.20)
project(FlappierBird VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Build Configuration
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Options
# ============================================================================

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_DOCS "Build documentation" ON)
option(ENABLE_TLA_VERIFICATION "Enable TLA+ verification" OFF)
option(ENABLE_Z3_VERIFICATION "Enable Z3 verification" OFF)
option(BUILD_SIMULATIONS "Build MATLAB/Octave simulations" OFF)

# ============================================================================
# Compiler Flags
# ============================================================================

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
        -O2
    )
endif()

# ============================================================================
# Source Files
# ============================================================================

set(ONBOARD_SOURCES
    arduino/onboard_active/onboard_active.ino
    arduino/onboard_active/controls.ino
    arduino/onboard_active/imu.ino
    arduino/onboard_active/radio.ino
    arduino/onboard_active/extras.ino
)

set(REMOTE_SOURCES
    arduino/remote_active/remote_active.ino
    arduino/remote_active/controls.ino
    arduino/remote_active/radio.ino
    arduino/remote_active/sticks.ino
    arduino/remote_active/prints.ino
    arduino/remote_active/extras.ino
)

# ============================================================================
# Include Directories
# ============================================================================

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino/onboard_active
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino/remote_active
)

# ============================================================================
# Testing
# ============================================================================

if(BUILD_TESTS)
    enable_testing()
    
    # Find Unity test framework
    find_package(Unity)
    
    if(Unity_FOUND)
        add_subdirectory(tests)
        
        # Add test targets
        add_test(NAME quaternion_tests COMMAND test_quaternion)
        add_test(NAME sensor_fusion_tests COMMAND test_sensor_fusion)
        add_test(NAME control_tests COMMAND test_control)
    else()
        message(WARNING "Unity test framework not found. Tests will not be built.")
    endif()
endif()

# ============================================================================
# Documentation
# ============================================================================

if(BUILD_DOCS)
    find_package(Doxygen)
    
    if(DOXYGEN_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        add_custom_target(docs ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
        
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs/html
                DESTINATION share/doc/flappierbird
                OPTIONAL)
    else()
        message(WARNING "Doxygen not found. Documentation will not be generated.")
    endif()
endif()

# ============================================================================
# Formal Verification
# ============================================================================

if(ENABLE_TLA_VERIFICATION)
    find_program(TLC tlc)
    
    if(TLC)
        add_custom_target(verify_tla
            COMMAND java -cp ${CMAKE_SOURCE_DIR}/tools/tla2tools.jar tlc2.TLC FlappierBird.tla
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/verification/tla
            COMMENT "Running TLA+ verification"
        )
    else()
        message(WARNING "TLA+ tools not found. TLA+ verification disabled.")
    endif()
endif()

if(ENABLE_Z3_VERIFICATION)
    find_program(Z3 z3)
    
    if(Z3)
        add_custom_target(verify_z3
            COMMAND ${Z3} FlappierBird.smt2
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/verification/z3
            COMMENT "Running Z3 constraint verification"
        )
        
        # Add as dependency of main target
        add_dependencies(verify_all verify_z3)
    else()
        message(WARNING "Z3 not found. Z3 verification disabled.")
    endif()
endif()

# Combined verification target
if(ENABLE_TLA_VERIFICATION OR ENABLE_Z3_VERIFICATION)
    add_custom_target(verify_all
        COMMENT "Running all formal verification"
    )
    
    if(ENABLE_TLA_VERIFICATION AND TLC)
        add_dependencies(verify_all verify_tla)
    endif()
endif()

# ============================================================================
# Simulations
# ============================================================================

if(BUILD_SIMULATIONS)
    find_program(OCTAVE octave)
    
    if(OCTAVE)
        add_custom_target(simulations
            COMMAND ${OCTAVE} --eval "run('torques.m')"
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/matlab
            COMMENT "Running MATLAB/Octave simulations"
        )
    else()
        message(WARNING "Octave not found. Simulations will not be built.")
    endif()
endif()

# ============================================================================
# Installation
# ============================================================================

install(FILES
    docs/RESEARCH_DEVELOPMENT_REPORT.md
    README.md
    DESTINATION share/doc/flappierbird
)

install(DIRECTORY
    verification/
    DESTINATION share/flappierbird/verification
    FILES_MATCHING PATTERN "*.tla" PATTERN "*.smt2"
)

# ============================================================================
# Package Configuration
# ============================================================================

include(CMakePackageConfigHelpers)

set(CONFIG_INSTALL_DIR lib/cmake/FlappierBird)

install(EXPORT FlappierBirdTargets
    FILE FlappierBirdTargets.cmake
    NAMESPACE FlappierBird::
    DESTINATION ${CONFIG_INSTALL_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlappierBirdConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/FlappierBirdConfig.cmake
    INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/FlappierBirdConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/FlappierBirdConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/FlappierBirdConfigVersion.cmake
    DESTINATION ${CONFIG_INSTALL_DIR}
)

# ============================================================================
# Summary
# ============================================================================

message(STATUS "")
message(STATUS "FlappierBird Configuration Summary")
message(STATUS "==================================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Options:")
message(STATUS "  BUILD_TESTS: ${BUILD_TESTS}")
message(STATUS "  BUILD_DOCS: ${BUILD_DOCS}")
message(STATUS "  ENABLE_TLA_VERIFICATION: ${ENABLE_TLA_VERIFICATION}")
message(STATUS "  ENABLE_Z3_VERIFICATION: ${ENABLE_Z3_VERIFICATION}")
message(STATUS "  BUILD_SIMULATIONS: ${BUILD_SIMULATIONS}")
message(STATUS "")
