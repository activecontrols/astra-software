#include "SDCard.h"
#include "Router.h"
#include "portenta_pins.h"
#include <SD.h>

bool SDCard::begin() {
  if (!SD.begin(SD_CARD_CS)) {
    Router::println("No SD Card detected - continuing anyway...");
    return false;
  }

  Router::add({ls, "ls"});
  Router::add({rm, "rm"});
  Router::add({cat, "cat"});
  return true;
}

File SDCard::open(const char *filename, char mode) {
  if (strlen(filename) > 12) {
    // https://www.if.ufrj.br/~pef/producao_academica/artigos/audiotermometro/audiotermometro-I/bibliotecas/SdFat/Doc/html/
    Router::print("Error opening file <");
    Router::print(filename);
    Router::println(">, file names must be 12 characters or less.");
    return File(); // null file
  }
  File f = SD.open(filename, mode);

  if (!f) {
    Router::println("Error opening file - returning null file.");
  }
  return f;
}

void SDCard::ls(const char *) {
  String result = "";
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      break;
    }
    result += entry.name();
    result += " ";
    entry.close();
  }
  root.close();
  Router::println(result.c_str());
}

void SDCard::rm(const char *filename) {
  if (filename == nullptr || strlen(filename) == 0) {
    Router::print("Enter filename: ");
    String fname = Router::read(50);
    filename = (char *)fname.c_str();
  }
  if (SD.remove(filename)) {
    Router::println("File removed.");
  } else {
    Router::println("File not found.");
  }
}

void SDCard::cat(const char *filename) { // okay technically this can only print one file at a time, so its not a real `cat`
  if (filename == nullptr || strlen(filename) == 0) {
    Router::print("Enter filename: ");
    filename = Router::read(50).c_str();
  }
  File f = SD.open(filename, FILE_READ);
  if (f) {
    while (f.available()) {
      Serial.println(f.readStringUntil('\n'));
    }
    f.close();
  } else {
    Router::println("File not found.");
  }
}

void SDCard::write_bytes(const char *filename, const uint8_t *data, unsigned int len) {
  File f = SD.open(filename, FILE_WRITE | O_TRUNC | O_CREAT);
  if (!f) {
    Router::println("Error opening file for writing. Try shorter filename.");
    return;
  }
  f.write(data, len);
  f.close();
}

void SDCard::load_bytes(const char *filename, uint8_t *data, unsigned int len) {
  File f = SD.open(filename, FILE_READ);
  if (!f) {
    Router::println("File not found.");
    return;
  }
  if (f.size() != len) {
    Router::println("File size does not match expected length.");
    f.close();
    return;
  }
  f.read(data, len);
  f.close();
}