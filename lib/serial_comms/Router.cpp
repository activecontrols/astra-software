#include "Router.h"

#include "CString.h"
//#include "SDCard.h"

#define COMMAND_BUFFER_SIZE (200)

// TODO: IG think about thread safety since args are global

namespace Router {
//File comms_log_file;

CString<COMMAND_BUFFER_SIZE> commandBuffer; // TODO: IG think about if this should be a raw char array? we don't use append or anything fancy
char *argStart = nullptr;                   // points to the start of the arguments in commandBuffer, or is null if no args

namespace { // private namespace
vector<func> funcs;

void readCommand() {
  // read until newline char or 200 characters (hopefully none of our funcs have names that long lol)
  size_t cmd_length = COMMS_SERIAL.readBytesUntil('\n', commandBuffer.str, COMMAND_BUFFER_SIZE - 1);
  commandBuffer.str[cmd_length] = '\0'; // null terminate
  commandBuffer.trim();                 // remove leading/trailing whitespace or newline

  // find first space to separate command from args
  argStart = strchr(commandBuffer.str, ' ');
  if (argStart != nullptr) {
    *argStart = '\0'; // null terminate the command name
    argStart++;       // point to the start of the args
    while (*argStart == ' ') {
      argStart++; // skip leading spaces
    }
    // if (*argStart == '\0') { // this case should not happen since vincent's trim should remove trailing spaces
    //   argStart = nullptr; // if only spaces after command, treat as no args
    // }
  }


  // comms_log_file.print("<");
  // comms_log_file.print(commandBuffer.str);
  // comms_log_file.print(" ");
  // if (argStart != nullptr) {
  //   comms_log_file.print("args: ");
  //   comms_log_file.print(argStart);
  // }
  // comms_log_file.print(">\n");
  // comms_log_file.flush();
}
} // namespace

void begin() {
  COMMS_SERIAL.begin(COMMS_RATE);
  COMMS_SERIAL.setTimeout((unsigned long)-1); // wrap around to max long so we never time out

  // if (SDCard::begin()) {
  //   comms_log_file = SDCard::open("log.txt", FILE_WRITE);
  // } else {
  //   Router::println("SD card not found.");
  //   while (true) {
  //     Router::println("Reboot once SD card inserted...");
  //     delay(1000);
  //   }
  // }
}

void println(const char *msg) {
  COMMS_SERIAL.println(msg);
  // comms_log_file.println(msg);
  // comms_log_file.flush();
}

void print(const char *msg) {
  COMMS_SERIAL.print(msg);
  // comms_log_file.print(msg);
  // comms_log_file.flush();
}

void send(char msg[], unsigned int len) {
  COMMS_SERIAL.write(msg, len);
}

void receive(char msg[], unsigned int len) {
  COMMS_SERIAL.readBytes(msg, len);
}

String read(unsigned int len) {
  // String s = COMMS_SERIAL.readStringUntil('\n', len);
  String s = COMMS_SERIAL.readStringUntil('\n'); // TODO: len no longer supported?
  s.trim();                                      // remove leading/trailing whitespace or newline

  // comms_log_file.print("<");
  // comms_log_file.print(s);
  // comms_log_file.print(">\n");
  // comms_log_file.flush();

  return s;
}

void add(func f) {
  funcs.push_back(f);
}

[[noreturn]] void run() { // attribute here enables dead-code warning & compiler optimization

  while (true) {
    readCommand();

    // commandBuffer.print(); // not needed with echo

    bool cmd_found = false;
    for (auto &f : funcs) {
      if (commandBuffer.equals(f.name)) {
        // f.f(); // call the function. it can decide to send, receive or whatever.
        f.f(argStart); // call the function. it can decide to send, receive or whatever.
        cmd_found = true;
        break;
      }
    }
    if (!cmd_found) {
      println("Command not found.");
    }
  }
}

void print_all_cmds() {
  println("All commands: ");
  for (auto &f : funcs) {
    println(f.name);
  }
}
} // namespace Router