#pragma once

#include <SD.h>

namespace SDCard {
bool begin();
File open(const char *filename, char mode);

void ls(const char*);
void rm(const char*);
void cat(const char*);
}; // namespace SDCard
