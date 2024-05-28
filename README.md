# About
Most DIY space mouse designs use Arduino pro micro based on ATmega32U4. Sice this AVR microcontrollers are more than 20 years old, I've decided to port firmware to newer  MCU, namely Espressif ESP32-S2. Besides lower cost it provides also ADC 100x higher sampling rate and better precision , which should reduce jitter while reading position from analog joysticks.

Still work in progress. Analog part works, device is correctly detected by system and 3Dconnexion drivers, but for some reason, messages from device are either ignored or not correctly interpreted by driver.
