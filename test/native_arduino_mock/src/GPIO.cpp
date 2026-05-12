#include "Arduino.h"
#include <chrono>
#include <thread>

bool pin_state[144];

void digitalWrite(int pin, int mode) {
  pin_state[pin] = mode;
}

void pinMode(int, int) {};

std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
void delay(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
void delayMicroseconds(int ms) {
  std::this_thread::sleep_for(std::chrono::microseconds(ms));
}

uint32_t millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}
uint32_t micros() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}