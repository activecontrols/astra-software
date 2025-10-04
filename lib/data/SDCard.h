// #pragma once

// #include <SD.h>

// namespace SDCard {
// bool begin();
// File open(const char *filename, char mode);

void ls(const char *);
void rm(const char *);
void cat(const char *);

void write_bytes(const char *filename, const uint8_t *data, unsigned int len);
void load_bytes(const char *filename, uint8_t *data, unsigned int len);
}; // namespace SDCard
