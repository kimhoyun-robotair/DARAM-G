cmake_minimum_required(VERSION 3.8)
project(frontier_msgs)

# ─── 컴파일 옵션 ──────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── 의존 패키지 ─────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ─── 메시지 목록 ─────────────────────────────────────────
set(msg_files
  msg/FrontierInfo.msg          # 단일 프런티어
  msg/FrontierInfoArray.msg     # 배열(헤더 + FrontierInfo[])
)

# ─── 인터페이스 생성 ────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# ─── 테스트 / Lint (선택) ───────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
