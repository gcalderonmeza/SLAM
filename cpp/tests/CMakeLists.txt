include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
# Prevent GoogleTest from overriding our compiler/linker settings on Windows.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(slam_tests
    test_motion_model.cpp
    test_sensor_model.cpp
    test_algorithm.cpp
)
target_link_libraries(slam_tests PRIVATE slam GTest::gtest_main)
if(MSVC)
    target_compile_options(slam_tests PRIVATE /W4)
else()
    target_compile_options(slam_tests PRIVATE -Wall -Wextra)
endif()

include(GoogleTest)
gtest_discover_tests(slam_tests)
