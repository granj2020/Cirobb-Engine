# Version minima requerida para generar este script
cmake_minimum_required(VERSION 3.8)


# Crea el proyecto principal llamado 'Cirobb Engine' Solucion
project(CirobbEngine VERSION 1.1.6) 


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


find_package(OpenGL REQUIRED)
add_subdirectory(freeglut)
add_subdirectory(cirobb)
add_subdirectory(test)

