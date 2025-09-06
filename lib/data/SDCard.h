#pragma once

#include <SD.h>

namespace SDCard {
bool begin();
File open(const char *filename, char mode);

void ls();
void rm();
void cat();
}; // namespace SDCard
