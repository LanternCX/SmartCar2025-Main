# 包含交叉编译工具链
include(cross.cmake)

# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_STANDARD 17)
# 对于 C++ 编译器
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")
 # 对于 C 编译器
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")

# 获取当前 CMakeLists.txt 文件所在目录的完整路径
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})

# 获取上上级目录的完整路径
get_filename_component(PARENT_PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)

# 从完整路径中提取文件夹名称
get_filename_component(PARENT_PARENT_FOLDER_NAME ${PARENT_PARENT_DIR} NAME)

# 打印上上级文件夹名称
message("Parent parent folder name: ${PARENT_PARENT_FOLDER_NAME}")

# 定义项目名称和版本，并指定使用C和C++语言
project(${PARENT_PARENT_FOLDER_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 设置OpenCV的安装路径
# set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
set(CMAKE_PREFIX_PATH "/opt/loongarch64/opencv") 

# 查找OpenCV库，确保安装了所需的依赖
find_package(OpenCV REQUIRED)

# 包含OpenCV的头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# 包含项目的自定义库路径
include_directories(../main)
include_directories(../../libraries/zf_common)
include_directories(../../libraries/zf_device)
include_directories(../../libraries/zf_driver)
include_directories(../../libraries/zf_components)

include_directories(../main/utils)
include_directories(../main/utils/display)
include_directories(../main/utils/control)
include_directories(../main/utils/math)
include_directories(../main/utils/time)
include_directories(../main/utils/debug)
include_directories(../main/image)
include_directories(../main/vision)
include_directories(../main/control)

include_directories(../main/inc)
include_directories(../main/src)

# 从目录下查找C/C++文件
aux_source_directory(../main DIR_SRCS)
aux_source_directory(../../libraries/zf_common DIR_SRCS)
aux_source_directory(../../libraries/zf_device DIR_SRCS)
aux_source_directory(../../libraries/zf_driver DIR_SRCS)
aux_source_directory(../../libraries/zf_components DIR_SRCS)

aux_source_directory(../main/utils DIR_SRCS)
aux_source_directory(../main/utils/display DIR_SRCS)
aux_source_directory(../main/utils/control DIR_SRCS)
aux_source_directory(../main/utils/math DIR_SRCS)
aux_source_directory(../main/utils/time DIR_SRCS)
aux_source_directory(../main/utils/debug DIR_SRCS)
aux_source_directory(../main/vision DIR_SRCS)
aux_source_directory(../main/control DIR_SRCS)
aux_source_directory(../main/image DIR_SRCS)

aux_source_directory(../main/inc DIR_SRCS)
aux_source_directory(../main/src DIR_SRCS)

# 创建可执行文件，使用项目名称作为目标名称
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# 链接OpenCV库到可执行文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})