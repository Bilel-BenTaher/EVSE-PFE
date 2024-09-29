EVSE (Electric Vehicle Supply Equipment) Development Project
This repository contains all the files related to my final year project for the development of Electric Vehicle Supply Equipment (EVSE). The project includes hardware schematics, design files, and firmware for an STM32U575VGT6 microcontroller, along with peripheral device drivers and task management through FreeRTOS.

Project Structure
/Altium/: Contains the Altium project files for the EVSE circuit design. This folder includes the EVSE schematic.

/Design/: Contains files related to the design of the EVSE system, including flowcharts, general functional block diagrams, and detailed diagrams of the internal EVSE components.

/core/: The firmware code written for the STM32U575VGT6. This folder contains all source files, including header and C files for various sensors and peripherals.

Sensors:

ds1621.h & ds1621.c: Driver for the DS1621 temperature sensor.
pzem004t.h & pzem004t.c: Driver for the PZEM004T sensor, used to measure voltage, current, and power.

Display:

oled.h & oled.c: Driver for the OLED display to show real-time data.
fonts.h: Defines the fonts used for text display on the OLED screen.

RTC:

rtc.h & rtc.c: Code to manage the real-time clock for date and time display.
LEDs & Diodes:

led.h: Macros and functions for controlling status LEDs.

FreeRTOS:

The FreeRTOS configuration and task definitions to manage system processes, including sensor communication, error handling, and display updates.

Helper Functions:

help.h: Contains utility functions that are reused across different parts of the codebase.
