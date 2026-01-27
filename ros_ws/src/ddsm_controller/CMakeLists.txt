cmake_minimum_required(VERSION 3.8)
project(ddsm_controller)

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
# Boostのsystemコンポーネントを探索
find_package(Boost REQUIRED COMPONENTS system)

# インクルードパスの設定 (ヘッダーファイルが入っている場所)
include_directories(include)

# 実行可能ファイルの作成
add_executable(ddsm_controller_node 
  src/ddsm_controller.cpp
)

# ROS 2 関連の依存関係をリンク
ament_target_dependencies(ddsm_controller_node
  rclcpp
  std_msgs
)

# Boost ライブラリを明示的にリンク
target_link_libraries(ddsm_controller_node
  Boost::system
)

# インストール設定 (ros2 run で実行できるようにする)
install(TARGETS
  ddsm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)


# amentの設定
ament_package()



# cmake_minimum_required(VERSION 3.8)
# project(DDSMController)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()


