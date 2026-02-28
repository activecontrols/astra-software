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
  Router::print("WARNING: Proceeding will permanently delete all data on the flash.\n");
  Router::printf("Enter %d to proceed: ", key);

  char* res = Router::readline();

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
    memcpy(write_buf + buf_index, data + data_index, write_count);
    data_index += write_count;
    buf_index += write_count;

    // if write buffer is full, send all
    if (buf_index >= sizeof(write_buf)) {
      // write data to a new page in the flash
      if (!Flash::page_program(program_addr, write_buf, sizeof(write_buf))) {
        Router::print("Page Program Failed\n");
        return;
      }
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

  Flash::wait_for_wip(1500);

  log_enable = false;
  return;
}

void flash_test() {
  if (!log_enable) {
    Router::println("Log is disabled. Use command log_arm before running the flash test.");
    return;
  }
  static uint8_t write_buf[PAGE_SIZE * 30 + 213];
  for (int i = 0; i < sizeof(write_buf); i++) {
    // write_buf[i] = rand();
    write_buf[i] = (i % 10) + '0';
  }

  Router::printf("\nWriting %d bytes to flash.\n", sizeof(write_buf));

  unsigned long start_time = micros();

  write(write_buf, sizeof(write_buf));
  complete();

  unsigned long delta = micros() - start_time;

  Router::println("\nWrite complete.");
  Router::printf("Time taken (ms): %.2lf\nWrite Speed: %.2lf B/s\n\n", delta * 1e-3, sizeof(write_buf) * 1e6 / delta);

  // now read back the data and compare

  static uint8_t read_buf[PAGE_SIZE];

  Router::println("Reading data back to verify integrity...");

  for (uint32_t addr = 0; addr < sizeof(write_buf); addr += PAGE_SIZE) {
    Flash::read(addr, PAGE_SIZE, read_buf);

    // compare the response to the programmed data
    size_t compare_count = min((unsigned long)PAGE_SIZE, sizeof(write_buf) - addr);

    if (memcmp(read_buf, write_buf + addr, compare_count)) {
      Router::printf("Data comparison failed. Page address: %X\n", addr);

      for (int i = 0; i < sizeof(read_buf); ++i) {
        Router::print(read_buf[i]);
        Router::print(' ');
      }

      Router::println();
      return;
    }
  }

  Router::printf("Flash Read/Write test succeeded!\nBytes written/read back: %d\n", sizeof(write_buf));

  return;
}

void begin() {
  Flash::begin();

  Router::add({cmd_log_arm, "log_arm"});
  Router::add({flash_test, "flash_test"});
}

}; // namespace Logging