cmake_minimum_required(VERSION 3.8)
project(pig_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

# 導出資源資料夾
install(DIRECTORY
  models
  media
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
