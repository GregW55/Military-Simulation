cmake_minimum_required(VERSION 3.31)
project(ECS_sim)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES}
        src/models/ScenarioData.h
        src/utils/ScenarioParser.cpp
        src/utils/ScenarioParser.h
        src/utils/TacticalDataLoader.cpp
        src/utils/TacticalDataLoader.h
        src/core/AegisEngine.cpp
        src/core/AegisEngine.h
        src/utils/ThreatAnalysis.h
        src/utils/MetricsLogger.h
        src/ecs/systems/RadarSystem.cpp
        src/ecs/systems/CombatSystem.cpp
        src/ecs/systems/GuidanceSystem.cpp
        src/ecs/systems/MovementSystem.cpp
        src/ecs/systems/ProximityFuseSystem.cpp
        src/ecs/Systems.cpp
        src/ecs/Events.h
        src/ecs/systems/LoggingSystem.cpp
)


# Assuming entt.hpp is located at external/entt.hpp
target_include_directories(${PROJECT_NAME} PRIVATE
        external
        ${raylib_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        raylib
        nlohmann_json::nlohmann_json
)

# Find the directory where the C++ compiler (and its DLLs) lives
get_filename_component(MINGW_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)

# Automatically copy all required MinGW runtime DLLs to the build output directory
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