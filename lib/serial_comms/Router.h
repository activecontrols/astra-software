#pragma once

#include "portenta_pins.h"
#include <Arduino.h>
#include <SD.h>
#include <functional>
#include <string>
#include <vector>

using namespace std;

// #define USE_RADIO

struct func;
struct func_no_args;

namespace Router {

#ifdef USE_RADIO
extern UART radio_uart;
#define COMMS_SERIAL radio_uart
#define COMMS_RATE 57600
#endif

#ifndef USE_RADIO
#define COMMS_SERIAL Serial
#define COMMS_RATE 115200
#endif

extern File comms_log_file;

// initializes the serial port and configures logs
void begin();

inline void println() {
  COMMS_SERIAL.println();
  comms_log_file.println();
  comms_log_file.flush();
}

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

template <typename T> void print(T value, int prec_or_base) {
  COMMS_SERIAL.print(value, prec_or_base);
  comms_log_file.print(value, prec_or_base);
  comms_log_file.flush();
}

template <typename T> void println(T value, int prec_or_base) {
  COMMS_SERIAL.println(value, prec_or_base);
  comms_log_file.println(value, prec_or_base);
  comms_log_file.flush();
}

template <typename... Args> void printf(const char *format, Args... args) {
  char buffer[200];
  snprintf(buffer, sizeof(buffer), format, args...);
  print(buffer);
}

template <typename T> void mprint(const T &t) { // base case
  print(t);
}

// "multi-print"
template <typename T, typename... Args> void mprint(const T &t, const Args &...args) { // recursive case. all of this is done at compile time.
  print(t);
  mprint(args...);
}

template <typename... Args> void mprintln(const Args &...args) {
  mprint(args...);
  println();
}

// could define a mprint with separator but honestly printf / just manually including sep is fine.

// send sends raw bytes over the serial port. the caller is responsible for
// freeing the memory of the message
void send(char msg[], unsigned int len);

// receive reads raw bytes from the serial port into the supplied buffer.
// the caller is responsible for freeing the memory of the message
void receive(char msg[], unsigned int len);

// reads a message from the serial port into a string and returns it
char *read();

#define MAX_PACKET_SIZE 256
extern char compressed_data[MAX_PACKET_SIZE];

template <typename S> void print_compressed(const char *id, S data) {
  print(id); // each message starts with an ID, and ends with a newline

  // need an extra byte for every 7 bits
  int extra_bytes_needed = (sizeof(data) + 6) / 7; // equivalent to ceil(size / 7)
  char *raw_data = (char *)&data;                  // allows [] index

  for (size_t i = 0; i < sizeof(data); i += 7) {
    compressed_data[sizeof(data) + i / 7] = 0x80; // wipe all the extra bytes
  }

  for (size_t i = 0; i < sizeof(data); i++) {
    compressed_data[i] = raw_data[i] | 0x80; // set all the upper bits
    if (raw_data[i] & 0x80) {
      compressed_data[sizeof(data) + i / 7] |= 1 << (6 - (i % 7));
    }
  }

  send(compressed_data, sizeof(data) + extra_bytes_needed);

  println();
}

// add registers a new function to the router
void add(func f);
void add(func_no_args fna);

// run starts monitoring the serial port for messages and calls the
// appropriate function when a message is received. this function never
// returns.
[[noreturn]] extern void run();

// for help function
void print_all_cmds();
// replacement for scanf
bool parse_doubles(const char *, double *vals, int count);

}; // namespace Router

struct func {
  std::function<void(const char *)> f;
  const char *name;
};

struct func_no_args {
  std::function<void()> f;
  const char *name;
};
