cmake_minimum_required(VERSION 3.8)
project(mecanum_controller)

# C++規格の指定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# コンパイルオプション（警告を表示するように設定）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージの探索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Boostのsystemコンポーネントを探索
find_package(Boost REQUIRED COMPONENTS system)

# インクルードパスの設定
include_directories(include)

# 実行可能ファイルの作成
add_executable(mecanum_controller_node
  src/mecanum_controller.cpp
)

# ROS 2 関連の依存関係をリンク
ament_target_dependencies(mecanum_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Boost ライブラリを明示的にリンク
target_link_libraries(mecanum_controller_node
  Boost::system
)

# インストール設定
install(TARGETS
  mecanum_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
