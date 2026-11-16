# PCA9685 reference board — Adafruit 815

Official manufacturer models: reference.step and reference.stl. Source URLs and SHA256 hashes are in source.json. These are the Adafruit board, not a universal PCA9685 footprint or a verified model of the purchased board.

Official documentation: https://learn.adafruit.com/16-channel-pwm-servo-driver/downloads
Pin interfaces: https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts

Product page dimensions exclude headers and terminal block (62.5 × 25.4 × 3 mm). Downloaded STL bounds span about 62.23 × 25.4 × 12.1 mm, including connector geometry. Preserve the model coordinate frame. Actual purchased-board matching and connector insertion clearance remain pending. Official mounting hole diameter is 2.5 mm; do not use the shell's M3 screws through this PCB.

Pico design uses 3.3 V VCC and common GND, SDA/SCL, 16 PWM outputs; default address 0x40. VPLUS is an independent servo supply, not a regulator output. Wiring allocation through this driver is not yet implemented in the reference provider.
