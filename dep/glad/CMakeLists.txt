cmake_minimum_required(VERSION 3.10)

project(glad VERSION 1.0)

add_library(glad src/glad.c)

target_include_directories(glad PUBLIC include/)