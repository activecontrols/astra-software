#pragma once
#include <cstdint>

class Serial {
public:
virtual bool is_open() = 0;

// return 0 on error
virtual int write(const char* data, unsigned int len, bool end_with_newline = false) = 0;

// return # of bytes read, -1 on error
virtual int read(char* data, unsigned int max_len) = 0;

virtual void close() = 0;
};