#pragma once

#include <Arduino.h>
#include <SD.h>
#include <functional>
#include <string>
#include <vector>

using namespace std;

#define COMMS_SERIAL Serial
#define COMMS_RATE 115200

struct func;

namespace Router {

extern File comms_log_file;

// initializes the serial port and configures logs
void begin();

template <typename T> void println(T value) {
  COMMS_SERIAL.println(value);
  comms_log_file.println(value);
  comms_log_file.flush();
}

template <typename T> void print(T value) {
  COMMS_SERIAL.print(value);
  comms_log_file.print(value);
  comms_log_file.flush();
}

template <typename T> void println(T value, int prec_or_base) {
  COMMS_SERIAL.println(value, prec_or_base);
  comms_log_file.println(value, prec_or_base);
  comms_log_file.flush();
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
