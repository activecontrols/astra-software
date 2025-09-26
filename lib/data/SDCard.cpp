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
  return SD.open(filename, mode);
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