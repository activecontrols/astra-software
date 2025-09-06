#include <SD.h>
#include "SDCard.h"
#include "Router.h"

bool SDCard::begin() {
  if (!SD.begin()) { // TODO RJN - set the SPI pins for the SD Card
    Router::info("No SD Card detected - continuing anyway...");
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

void SDCard::ls() {
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
  Router::info(result.c_str());
}

void SDCard::rm() {
  Router::info_no_newline("Enter filename: ");
  String filename = Router::read(50);
  if (SD.remove(filename.c_str())) {
    Router::info("File removed.");
  } else {
    Router::info("File not found.");
  }
}

void SDCard::cat() {
  Router::info_no_newline("Enter filename: ");
  String filename = Router::read(50);
  File f = SD.open(filename.c_str(), FILE_READ);
  if (f) {
    while (f.available()) {
      Serial.println(f.readStringUntil('\n'));
    }
    f.close();
  } else {
    Router::info("File not found.");
  }
}