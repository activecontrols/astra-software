#include "CString.h"

// append a char* to a char array
int cstring::append(char *str, size_t bufferLen, const char *src) {
  size_t availableSpace = bufferLen - strlen(str) - 1;
  size_t srcLen = strlen(src);
  strncat(str, src, availableSpace);
  int leftover = srcLen > availableSpace ? srcLen - availableSpace : 0;
  return leftover;
}

// append a double to a char array
int cstring::append(char *str, size_t bufferLen, double value, int precision) {
  size_t availableSpace = bufferLen - strlen(str) - 1;
  char *end = str + strlen(str);
  size_t written = snprintf(end, availableSpace + 1, "%.*g", precision, value);
  int leftover = written > availableSpace ? written - availableSpace : 0;
  return leftover;
}

// trim leading and trailing whitespace and new lines in a char array
void cstring::trim(char *str) {
  char *start = str;
  while (isspace((unsigned char)*start) || (*start == '\n')) {
    ++start;
  }

  char *end = start + strlen(start) - 1;
  while ((end > start) && (isspace((unsigned char)*end) || (*end == '\n'))) {
    --end;
  }

  memmove(str, start, end - start + 1);
  str[end - start + 1] = '\0';
}

// resolve a string containing backspaces to the user intent
// ie: '\babcd\bef' -> 'abcef'
void cstring::resolve_backspaces(char *str) {
  int valid_idx = 0;
  int current_idx = 0;
  while (str[current_idx] != '\0') {
    if (valid_idx != current_idx) {
      printf("Update %d to %d\n", valid_idx, current_idx);
      str[valid_idx] = str[current_idx];
    }

    if (str[current_idx] != '\b') { // backspace
      valid_idx++;
    } else {
      if (valid_idx > 0) { // ignore backspaces at start of string
        valid_idx--;
      }
    }
    current_idx++;
  }
  str[valid_idx] = '\0';
}