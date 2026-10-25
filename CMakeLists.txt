cmake_minimum_required(VERSION 3.31)
project(ECS_sim)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================================
# === DEPENDENCIES USED BY THE MAIN SIMULATION ===
# ================================================

include(FetchContent)

FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/master.tar.gz
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.12.0
)
FetchContent_MakeAvailable(json)

# ==================================
# === MAIN SIMULATION EXECUTABLE ===
# ==================================

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
        external
        ${raylib_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        raylib
        nlohmann_json::nlohmann_json
)

# ===================================================
# === MIN GW RUNTIME DLLS FOR THE MAIN SIMULATION ===
# ===================================================
get_filename_component(MINGW_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MINGW_BIN_DIR}/libwinpthread-1.dll"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MINGW_BIN_DIR}/libstdc++-6.dll"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MINGW_BIN_DIR}/libgcc_s_seh-1.dll"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

# ===================
# === GOOGLE TEST ===
# ===================

option(BUILD_TESTING "Build GoogleTest test targets" ON)

if(BUILD_TESTING)

    include(CTest)

    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )

    FetchContent_MakeAvailable(googletest)

    # --------------------------------------------------------
    # RADAR TEST EXECUTABLE
    # EXCLUDE_FROM_ALL means the normal build skips this target.
    # --------------------------------------------------------

    add_executable(RadarSystemTests EXCLUDE_FROM_ALL
        tests/RadarSystemTests.cpp

        src/ecs/systems/radar/SensorSystem.cpp
        src/ecs/systems/radar/TrackingSystem.cpp
        src/ecs/systems/radar/SensorFusionSystem.cpp

        src/core/GeoEngine.cpp
        src/utils/MathUtils.cpp
    )

    target_include_directories(RadarSystemTests PRIVATE
        src
        external
        ${raylib_SOURCE_DIR}/src
    )

    target_compile_features(RadarSystemTests PRIVATE cxx_std_23)

    target_link_libraries(RadarSystemTests PRIVATE
        GTest::gtest_main
        raylib
        nlohmann_json::nlohmann_json
    )

    include(GoogleTest)
    gtest_discover_tests(RadarSystemTests)

endif()
