#include "Router.h"

#include "CString.h"
#include "SDCard.h"

#define COMMAND_BUFFER_SIZE (200)

namespace Router {

UART radio_uart(RADIO_TX, RADIO_RX, NC, NC);

File comms_log_file;

CString<COMMAND_BUFFER_SIZE> commandBuffer;
CString<COMMAND_BUFFER_SIZE> readBuffer;

char *argStart = nullptr; // points to the start of the arguments in commandBuffer, or is null if no args

namespace { // private namespace
vector<func> funcs;

void readCommand() {
  // read until newline char or 200 characters (hopefully none of our funcs have names that long lol)
  size_t cmd_length = COMMS_SERIAL.readBytesUntil('\n', commandBuffer.str, COMMAND_BUFFER_SIZE - 1);
  commandBuffer.str[cmd_length] = '\0'; // null terminate
  commandBuffer.trim();                 // remove leading/trailing whitespace or newline
  commandBuffer.resolve_backspaces();

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

  comms_log_file.print("<");
  comms_log_file.print(commandBuffer.str);
  comms_log_file.print(" ");
  if (argStart != nullptr) {
    comms_log_file.print("args: ");
    comms_log_file.print(argStart);
  }
  comms_log_file.print(">\n");
  comms_log_file.flush();
}
} // namespace

void begin() {
  COMMS_SERIAL.begin(COMMS_RATE);
  COMMS_SERIAL.setTimeout((unsigned long)-1); // wrap around to max long so we never time out

  if (SDCard::begin()) {
    comms_log_file = SDCard::open("log.txt", FILE_WRITE);
  } else {
    Router::println("SD card not found.");
    while (true) {
      Router::println("Reboot once SD card inserted...");
      delay(1000);
    }
  }
}

void send(char msg[], unsigned int len) {
  COMMS_SERIAL.write(msg, len);
}

void receive(char msg[], unsigned int len) {
  COMMS_SERIAL.readBytes(msg, len);
}

char *read() {
  // read until newline char or 200 characters (hopefully none of our funcs have names that long lol)
  size_t read_length = COMMS_SERIAL.readBytesUntil('\n', readBuffer.str, COMMAND_BUFFER_SIZE - 1);
  readBuffer.str[read_length] = '\0'; // null terminate
  readBuffer.trim();                  // remove leading/trailing whitespace or newline
  readBuffer.resolve_backspaces();

  comms_log_file.print("<");
  comms_log_file.print(readBuffer.str);
  comms_log_file.print(">\n");
  comms_log_file.flush();

  return readBuffer.str;
}

char compressed_data[MAX_PACKET_SIZE];

void add(func f) {
  funcs.push_back(f);
}

void add(func_no_args fna) {
  func wrapped{[f = fna.f](const char *) { f(); }, // ignore the argument
               fna.name};
  funcs.push_back(wrapped);
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

bool parse_doubles(const char *str, double *vals, int count) { // sscanf doesnt handle doubles. 5 minutes of debugging resulted in that conclusion.
  const char *p = str;

  for (int i = 0; i < count; i++) {
    char *end;
    vals[i] = strtod(p, &end);

    if (end == p)
      return false;

    p = end;
  }

  return true;
}

} // namespace Router