#pragma once

#include <Arduino.h>
#include <functional>
#include <string>
#include <vector>
// #include <SD.h>

using namespace std;

#define COMMS_SERIAL Serial
#define COMMS_RATE 115200

struct func;

namespace Router {

// initializes the serial port and configures logs
void begin();

// info sends a string & newline over serial
void println(const char *msg);
void print(const char *msg);
inline void println(const String &msg) {
  println(msg.c_str());
}
inline void print(const String &msg) {
  print(msg.c_str());
}
inline void println(const std::string &msg) {
  println(msg.c_str());
}
inline void print(const std::string &msg) {
  print(msg.c_str());
}

// send sends raw bytes over the serial port. the caller is responsible for
// freeing the memory of the message
void send(char msg[], unsigned int len);

// receive reads raw bytes from the serial port into the supplied buffer.
// the caller is responsible for freeing the memory of the message
void receive(char msg[], unsigned int len);

// reads a message from the serial port into a string and returns it
String read(unsigned int len);

// add registers a new function to the router
void add(func f);

// run starts monitoring the serial port for messages and calls the
// appropriate function when a message is received. this function never
// returns.
[[noreturn]] extern void run();

// for help function
void print_all_cmds();

}; // namespace Router

struct func {
  std::function<void(const char *)> f;
  const char *name;
};
