file(GLOB_RECURSE FILES CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cc" "*.cpp" "*.h")
add_library(pure_pursuit  STATIC ${FILES})
target_include_directories(pure_pursuit 
  PUBLIC
    # BUILD 时：puresuitlib 自己的 include
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    # install 后 find_package 时：install 目录下的 include
    $<INSTALL_INTERFACE:include>
)

