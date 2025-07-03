cmake_minimum_required(VERSION 3.12)
project(pure_pursuit VERSION 0.1 LANGUAGES CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build: None, Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install"
      CACHE PATH "Install path prefix" FORCE)
endif()


# 自己的项目就别让它默认安装到系统目录去了
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install path prefix." FORCE)
endif()

set (CMAKE_CXX_FLAGS                     "${CMAKE_CXX_FLAGS} ${COMPILER_FLAGS}")
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO      "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3 ${DEBUG_INFO_FLAGS} ${CMAKE_CXX_FLAGS_ADD}")
set (CMAKE_CXX_FLAGS_DEBUG               "${CMAKE_CXX_FLAGS_DEBUG} -O0 ${DEBUG_INFO_FLAGS} -fno-inline ${CMAKE_CXX_FLAGS_ADD}")

# include(cmake/Target.cmake)
add_subdirectory(src)

option(PURSUIT_BUILD_DEMO "Build the pursuit demo application" ON)
if(PURSUIT_BUILD_DEMO AND CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  add_subdirectory("third_party/cubic_spline" EXCLUDE_FROM_ALL) 
  add_subdirectory(example)
endif()


# —— 安装静态库和头文件 —— 
install(TARGETS pure_pursuit
  EXPORT pure_pursuitTargets
  ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# —— 生成 version 文件 —— 
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/pure_pursuitConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)


if (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/pure_pursuitConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/pure_pursuit"
  )
endif()

# —— 导出 Targets 文件 —— 
install(EXPORT pure_pursuitTargets
  FILE pure_pursuitTargets.cmake
  NAMESPACE pure_pursuit::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pure_pursuit
)

# —— 安装 Config 与 version 文件 —— 
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/pure_pursuitConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/pure_pursuitConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pure_pursuit
)
