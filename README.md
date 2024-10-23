Electric Vehicle Supply Equipment (EVSE) Final Year Project
This repository contains all files related to my final year project, which focuses on the development of Electric Vehicle Supply Equipment (EVSE). The project encompasses hardware schematics, design files, and firmware for the STM32U575VGT6 microcontroller, as well as peripheral device drivers and task management using FreeRTOS.

Project Structure
The repository is organized as follows:

bash
Copy code
/Design/                # Contains design-related documents, including flowcharts, general functional block diagrams, and detailed diagrams of internal EVSE components.

/core/                  # Contains the firmware code for the STM32U575VGT6, including source files for various sensors and peripherals.
Core Firmware Components
Sensors:

ds1621.h & ds1621.c: Driver for the DS1621 temperature sensor.
pzem004t.h & pzem004t.c: Driver for the PZEM004T sensor, used for measuring voltage, current, and power.
Display:

oled.h & oled.c: Driver for the OLED display to show real-time data.
fonts.h: Contains definitions for fonts used on the OLED screen.
Real-Time Clock (RTC):

rtc.h & rtc.c: Code for managing the real-time clock for date and time display.
LEDs & Diodes:

led.h: Contains macros and functions for controlling status LEDs.
FreeRTOS:

Configuration and task definitions for managing system processes, including sensor communication, error handling, and display updates.
Helper Functions:

help.h: Utility functions that are reused across different parts of the codebase.
