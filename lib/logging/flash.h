#pragma once

#define SECTOR(addr) ((addr >> 12) & 0xFFF)
#define PAGE(addr) ((addr >> 8) & 0xFFFF)

namespace Flash {
void begin();

bool wait_for_wip(unsigned long max_delay = 50);

}; // namespace Flash