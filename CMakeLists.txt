cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

project(ZFSceneGraph LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

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

set(ZFSG_QT_ROOT "C:/Qt5/5.15.2/msvc2019_64" CACHE PATH "Qt installation root for Qt5")
if(ZFSG_QT_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${ZFSG_QT_ROOT}")
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

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


find_package(GTest CONFIG QUIET)

if(NOT GTest_FOUND)
    file(GLOB ZFSG_CONAN_GTEST_ROOTS "$ENV{USERPROFILE}/.conan2/p/b/gtest*/p")
    list(GET ZFSG_CONAN_GTEST_ROOTS 0 ZFSG_CONAN_GTEST_ROOT)

    if(ZFSG_CONAN_GTEST_ROOT AND EXISTS "${ZFSG_CONAN_GTEST_ROOT}/include/gtest/gtest.h")
        add_library(GTest::gtest STATIC IMPORTED)
        set_target_properties(GTest::gtest PROPERTIES
            IMPORTED_LOCATION "${ZFSG_CONAN_GTEST_ROOT}/lib/gtest.lib"
            INTERFACE_INCLUDE_DIRECTORIES "${ZFSG_CONAN_GTEST_ROOT}/include"
        )

        add_library(GTest::gtest_main STATIC IMPORTED)
        set_target_properties(GTest::gtest_main PROPERTIES
            IMPORTED_LOCATION "${ZFSG_CONAN_GTEST_ROOT}/lib/gtest_main.lib"
            INTERFACE_LINK_LIBRARIES GTest::gtest
        )

        set(GTest_FOUND TRUE)
    endif()
endif()

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

add_executable(zfsg_ptr_tests
    tests/Core/intrusive_ptr_tests.cpp
)

target_include_directories(zfsg_ptr_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_ptr_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
)

if(MSVC)
    target_compile_options(zfsg_ptr_tests PRIVATE /utf-8)
endif()

add_executable(zfsg_any_tests
    tests/Core/Any_tests.cpp
)

target_include_directories(zfsg_any_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_any_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
)

if(MSVC)
    target_compile_options(zfsg_any_tests PRIVATE /utf-8)
endif()

add_executable(zfsg_plane_tests
    tests/Core/Plane_tests.cpp
)

target_include_directories(zfsg_plane_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_plane_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(zfsg_plane_tests PRIVATE /utf-8)
endif()

add_executable(zfsg_sphere_tests
    tests/Core/Sphere_tests.cpp
)

target_include_directories(zfsg_sphere_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_sphere_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(zfsg_sphere_tests PRIVATE /utf-8)
endif()

add_executable(zfsg_aabb_tests
    tests/Core/AABB_tests.cpp
)

target_include_directories(zfsg_aabb_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_aabb_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(zfsg_aabb_tests PRIVATE /utf-8 /D_USE_MATH_DEFINES)
endif()

add_executable(zfsg_math_tests
    tests/Core/Math_tests.cpp
)

target_include_directories(zfsg_math_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_math_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
    zfsg_core_utils
)

if(MSVC)
    target_compile_options(zfsg_math_tests PRIVATE /utf-8 /D_USE_MATH_DEFINES)
endif()

add_executable(zfsg_frustum_tests
    tests/Core/Frustum_tests.cpp
)

target_include_directories(zfsg_frustum_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_frustum_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(zfsg_frustum_tests PRIVATE /utf-8 /D_USE_MATH_DEFINES)
endif()

add_executable(zfsg_rayintersect_tests
    tests/Core/RayIntersect_tests.cpp
)

target_include_directories(zfsg_rayintersect_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_rayintersect_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(zfsg_rayintersect_tests PRIVATE /utf-8)
endif()

add_executable(zfsg_lookat_tests
    tests/Core/LookAt_tests.cpp
)

target_include_directories(zfsg_lookat_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_lookat_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(zfsg_lookat_tests PRIVATE /utf-8)
endif()

add_executable(zfsg_scene_tests
    tests/SceneGraph_tests.cpp
)

target_include_directories(zfsg_scene_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(zfsg_scene_tests
    PRIVATE
    ${ZFSG_GTEST_MAIN_TARGET}
    Eigen3::Eigen
    zfsg_scene_graph
)

if(MSVC)
    target_compile_options(zfsg_scene_tests PRIVATE /utf-8)
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
gtest_discover_tests(zfsg_scene_tests)

