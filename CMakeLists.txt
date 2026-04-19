cmake_minimum_required(VERSION 3.16)          
project(tailor CXX)                           

set(CMAKE_CXX_STANDARD 20)                     
set(CMAKE_CXX_STANDARD_REQUIRED ON)            

option(TAILOR_BUILD_TESTS "Build tests" ON)


add_library(tailor INTERFACE)


target_include_directories(tailor INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/tailor-config.cmake.in   # 模板文件
    ${CMAKE_CURRENT_BINARY_DIR}/tailor-config.cmake      # 输出位置
    INSTALL_DESTINATION lib/cmake/tailor                  # 安装时的相对路径
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tailor-config.cmake
    DESTINATION lib/cmake/tailor)

install(TARGETS tailor EXPORT tailor-targets)
install(EXPORT tailor-targets FILE tailor-targets.cmake 
    DESTINATION lib/cmake/tailor)

install(DIRECTORY include/ DESTINATION include)

if(TAILOR_BUILD_TESTS)
    add_subdirectory(test)
endif()
