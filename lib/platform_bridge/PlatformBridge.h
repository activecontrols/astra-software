#ifndef PLATFORM_BRIDGE_H
#define PLATFORM_BRIDGE_H

#if defined(TARGET_TEENSY41) || defined(TARGET_PORTENTA_H7_M7)
#include <Arduino.h>
#include <SD.h>
#elif defined(TARGET_NATIVE)
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <istream>
#include <thread>
#endif

#if defined(TARGET_TEENSY41) || defined(TARGET_NATIVE)
#include <functional>
#include <string>
#include <vector>
#endif

/*
 * PlatformBridge.h
 *
 * Created on: 2025-06-30 by Ethan Chen
 * Maintained by Ethan Chen
 * Description: This file acts as an abstraction layer for the portions of the API used across multiple platforms
 */

// add csv and implement for tadpole sensors
// Make everything use CStrings if possible, but otherwise compensate with std::string derived class (yes ik it's bad
// practice) Move to multithreading (complete data no pausing unless explicitly required otherwise use fbo like buffer
// switching system and deprioritizing recentness of data)

#if defined(TARGET_TEENSY41)

// Not required, but list the portions of the APIs used here so we can keep track of things and move it to another
// namespace if needed
using ::delay;
using ::File;
using ::SD;
using ::Serial;
using ::String;

#elif defined(TARGET_NATIVE)

void *extmem_malloc(size_t size);
void extmem_free(void *ptr);
void *extmem_calloc(size_t nmemb, size_t size);
void *extmem_realloc(void *ptr, size_t size);

typedef bool boolean;

class String : public std::string {
public:
  String();
  String(const std::string &str);
  String(const char *s);
  String(const std::string &str, size_t pos, size_t len = std::string::npos);

  void trim();
};

extern std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
void delay(unsigned long ms);
void delayMicroseconds(unsigned long ms);
unsigned long millis();
unsigned long micros();

class usb_serial_class {
public:
  usb_serial_class();

  void begin(long bitsPerSecond);
  void setTimeout(long msTime);
  template <typename T> size_t print(T in);
  template <typename T> size_t println(T in);
  size_t write(const char *msg, size_t len);
  void flush();
  size_t readBytes(char *buffer, size_t length);
  String readString(size_t length = 120);
  size_t readBytesUntil(char terminator, char *buffer, size_t length);
  String readStringUntil(char terminator, size_t length = 120);
};

extern usb_serial_class Serial;

#define FILE_READ 0
#define FILE_WRITE 1

class File {
public:
  std::string name_str;
  std::filesystem::path path;
  char mode;
  std::fstream stream;
  std::filesystem::directory_iterator dir_cursor;
  inline const static std::filesystem::directory_iterator dir_end;

  File();
  File(const char *path_str, char mode = FILE_WRITE);

  operator bool() const;
  File openNextFile(uint8_t mode = 0);
  const char *name();
  void close();
  bool available();
  template <typename T> size_t print(T in);
  template <typename T> size_t println(T in);
  void flush();
  size_t write(char value);
  size_t write(const char *buffer, size_t length);
  int read();
  size_t read(void *buffer, size_t length);
  size_t readBytes(char *buffer, size_t length);
  String readString(size_t length = 120);
  size_t readBytesUntil(char terminator, char *buffer, size_t length, std::istream &stream = std::cin);
  String readStringUntil(char terminator, size_t length = 120);
};

class SDClass {
public:
  SDClass();

  bool begin(uint8_t csPin = 0);
  File open(const char *path_str, char mode = FILE_WRITE);
  bool exists(const char *file_path_str);
  bool remove(const char *file_path_str);
};

extern SDClass SD;

#endif

#endif

#include "PlatformBridge.tpp"