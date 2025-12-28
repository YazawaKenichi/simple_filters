cmake_minimum_required(VERSION 3.8)
project(filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xml から依存関係を読んで find_package
ament_auto_find_build_dependencies()

# ライブラリ関係 ament_target_dependencies
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/core/core.cpp
  )

# 実行ファイル関係 add_executable
ament_auto_add_executable(average
  src/average/main.cpp)
target_link_libraries(average ${PROJECT_NAME})

ament_auto_add_executable(median
  src/median/main.cpp)
target_link_libraries(median ${PROJECT_NAME})

ament_auto_add_executable(derivative
  src/derivative/main.cpp)
target_link_libraries(derivative ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install
ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
    launch
)

