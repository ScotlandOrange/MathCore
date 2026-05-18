# MathCore submodule — used as a subdirectory of ZFSceneGraph

# Eigen3 is already provided by the parent project
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 3.3 QUIET NO_MODULE)
    if(NOT Eigen3_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            eigen
            URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        )
        FetchContent_MakeAvailable(eigen)
        if(NOT TARGET Eigen3::Eigen)
            add_library(Eigen3::Eigen INTERFACE IMPORTED)
            target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
        endif()
    endif()
endif()

# Qt5 is already provided by the parent project
if(NOT TARGET Qt5::Core)
    find_package(Qt5 REQUIRED COMPONENTS Core)
endif()

add_library(zfsg_core_utils STATIC
    Core/Utils/Convert.cpp
    Core/Math/Distance.cpp
    Core/Math/Intersect.cpp
)

target_include_directories(zfsg_core_utils
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_core_utils
    PUBLIC
    Eigen3::Eigen
    Qt5::Core
)

if(MSVC)
    target_compile_options(zfsg_core_utils PRIVATE /utf-8)
endif()

# Tests
option(ZFSG_MATHCORE_BUILD_TESTS "Build MathCore tests" OFF)
if(ZFSG_MATHCORE_BUILD_TESTS)
    find_package(GTest CONFIG QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        )
        FetchContent_MakeAvailable(googletest)
    endif()

    if(TARGET GTest::gtest_main)
        set(ZFSG_GTEST_MAIN_TARGET GTest::gtest_main)
    elseif(TARGET gtest_main)
        set(ZFSG_GTEST_MAIN_TARGET gtest_main)
    else()
        message(FATAL_ERROR "GoogleTest main target was not found")
    endif()

    add_executable(zfsg_ptr_tests Tests/intrusive_ptr_tests.cpp)
    target_include_directories(zfsg_ptr_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_ptr_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET})
    if(MSVC)
        target_compile_options(zfsg_ptr_tests PRIVATE /utf-8)
    endif()

    add_executable(zfsg_any_tests Tests/Any_tests.cpp)
    target_include_directories(zfsg_any_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_any_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET})
    if(MSVC)
        target_compile_options(zfsg_any_tests PRIVATE /utf-8)
    endif()

    add_executable(zfsg_plane_tests Tests/Plane_tests.cpp)
    target_include_directories(zfsg_plane_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_plane_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen)
    if(MSVC)
        target_compile_options(zfsg_plane_tests PRIVATE /utf-8)
    endif()

    add_executable(zfsg_sphere_tests Tests/Sphere_tests.cpp)
    target_include_directories(zfsg_sphere_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_sphere_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen)
    if(MSVC)
        target_compile_options(zfsg_sphere_tests PRIVATE /utf-8)
    endif()

    add_executable(zfsg_aabb_tests Tests/AABB_tests.cpp)
    target_include_directories(zfsg_aabb_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_aabb_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen)
    if(MSVC)
        target_compile_options(zfsg_aabb_tests PRIVATE /utf-8 /D_USE_MATH_DEFINES)
    endif()

    add_executable(zfsg_math_tests Tests/Math_tests.cpp)
    target_include_directories(zfsg_math_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_math_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen zfsg_core_utils)
    if(MSVC)
        target_compile_options(zfsg_math_tests PRIVATE /utf-8 /D_USE_MATH_DEFINES)
    endif()

    add_executable(zfsg_frustum_tests Tests/Frustum_tests.cpp)
    target_include_directories(zfsg_frustum_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_frustum_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen)
    if(MSVC)
        target_compile_options(zfsg_frustum_tests PRIVATE /utf-8 /D_USE_MATH_DEFINES)
    endif()

    add_executable(zfsg_rayintersect_tests Tests/RayIntersect_tests.cpp)
    target_include_directories(zfsg_rayintersect_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_rayintersect_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen)
    if(MSVC)
        target_compile_options(zfsg_rayintersect_tests PRIVATE /utf-8)
    endif()

    add_executable(zfsg_lookat_tests Tests/LookAt_tests.cpp)
    target_include_directories(zfsg_lookat_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(zfsg_lookat_tests PRIVATE ${ZFSG_GTEST_MAIN_TARGET} Eigen3::Eigen)
    if(MSVC)
        target_compile_options(zfsg_lookat_tests PRIVATE /utf-8)
    endif()

    include(GoogleTest)
    gtest_discover_tests(zfsg_ptr_tests)
    gtest_discover_tests(zfsg_any_tests)
    gtest_discover_tests(zfsg_plane_tests)
    gtest_discover_tests(zfsg_sphere_tests)
    gtest_discover_tests(zfsg_aabb_tests)
    gtest_discover_tests(zfsg_math_tests)
    gtest_discover_tests(zfsg_frustum_tests)
    gtest_discover_tests(zfsg_rayintersect_tests)
    gtest_discover_tests(zfsg_lookat_tests)
endif()

