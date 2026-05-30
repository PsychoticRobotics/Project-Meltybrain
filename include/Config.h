//
// Created by Atharv Goel on 9/28/25.
// Updated last by Atharv Goel on 10/02/25.
//

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

#define PROTOCOL 1 // 0 = SPI, 1 = I2C

// IR mode:  0 = external beacons (IRBeaconTracker + IRSweep)
//           1 = beacon-free arena tracking (IRArenaTracker)
// In arena mode pins 2 and 3 are freed up — no external beacons needed.
#define IR_MODE  1

// Arena size in metres — only used when IR_MODE == 1.
// Common sizes: 8 ft = 2.44 m, 12 ft = 3.66 m, 16 ft = 4.88 m.
#define ARENA_SIZE_M  2.44f

#endif //MAIN_CONFIG_H
