#include "FlashLogging.h"
#include "Router.h"
#include "flash.h"

#include <string.h>

#include <random>

namespace Logging {

uint8_t write_buf[PAGE_SIZE];
int buf_index{0};
uint32_t program_addr{0};

bool log_enable = false;

void cmd_log_arm() {
  if (log_enable) {
    Router::print("Log is already enabled.\n");
    return;
  }
  const int key_min = 1000;
  const int key_max = 10000;
  srand(millis());
  int key = rand() % (key_max - key_min) + key_min;
  Router::print("WARNING: Proceeding will completely erase all data on the flash.\n");
  Router::printf("Enter %d to proceed: ", key);

  char res[21];
  int len = COMMS_SERIAL.readBytesUntil('\n', res, sizeof(res) - 1);
  res[len] = 0;

  if (atoi(res) != key) {
    Router::print("Incorrect key entered.\n");
    return;
  }

  Router::print("Erasing flash. This may take up to 1 minute.\n");

  if (!Flash::chip_erase()) {
    Router::print("Chip erase failed.\n");
    return;
  }

  Router::print("Flash erased.\n");
  Router::print("Logging enabled.\n");

  log_enable = true;
  buf_index = 0;
  program_addr = 0;
  return;
}

void write(uint8_t *data, unsigned int len) {
  if (!log_enable)
    return;

  unsigned int data_index = 0;

  while (data_index < len) {
    unsigned long write_count = len - data_index;
    if (write_count + buf_index > sizeof(write_buf)) {
      write_count = sizeof(write_buf) - buf_index;
    }
    // copy bytes into send buffer
    memcpy(write_buf + buf_index, data, write_count);
    data_index += write_count;
    buf_index += write_count;

    // if write buffer is full, send all
    if (buf_index >= sizeof(write_buf)) {
      // write data to a new page in the flash
      Flash::page_program(program_addr, write_buf, sizeof(write_buf));
      program_addr += sizeof(write_buf);
      buf_index = 0;
    }
  }
  return;
}

// empty the send buffer and disable logging
void complete() {
  if (!log_enable)
    return;

  if (buf_index == 0)
    return;

  Flash::page_program(program_addr, write_buf, buf_index);

  log_enable = false;
  return;
}

void begin() {
  Flash::begin();

  Router::add({cmd_log_arm, "log_arm"});
}
}; // namespace Logging