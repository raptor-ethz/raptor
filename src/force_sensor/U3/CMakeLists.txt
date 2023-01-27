cmake_minimum_required(VERSION 3.1.0)
project(FTSensor)

add_library( U3
  u3.c
  u3.h
  u3Streamer.cpp
  u3Streamer.h
)