This repository contains the firmware for an Automated Tensiometer based on the Wilhelmy Plate method. The system integrates coordinated dual-axis motion control and high-resolution data acquisition to measure the surface tension of liquids (mN/m).

Project Overview

The system acts as a specialized CNC controller for surface chemistry, receiving commands via Serial Interface to coordinate surface compression and substrate coating.

System Architecture:

X-Axis (Surface Compression): Controls the mobile barriers ("spazzole") to reduce the surface area, increasing the surface pressure of the molecular film.
Y-Axis (Dipping/Coating): Manages the vertical movement of the substrate for controlled immersion and withdrawal (coating process).
Surface Pressure Sensing: Utilizes a Wilhelmy Plate attached to a high-resolution load cell (HX711) to monitor surface pressure in real-time.

Key Features:

Coordinated Motion: Precision control of stepper motors for constant-rate compression and dipping.
Wilhelmy Plate Calibration: Integrated linear regression to convert sensor voltage into surface pressure (mN/m).
Safety Interlocks: 4 limit switches to define the mechanical boundaries of the barriers and the dipping arm.
Interactive Serial CLI: Comprehensive command set for manual positioning, homing, and automated measurement cycles.

Scientific Logic:
The firmware calculates the surface pressure (pi) by measuring the change in force (F) acting on the Wilhelmy plate.

Hardware Stack

Controller: Arduino Mega
Drivers: A4988 / DRV8825 Stepper Drivers
Sensors: HX711 Load Cell Amplifier + Wilhelmy Plate
Limits: 4x Mechanical/Optical Endstops

Serial commands:
'w' and 's' move Y axis
'a' and 'd' move X axis
'A' and 'D' move both axes;
'p' performs weighting (old method);
'x' and 'y' set motion ratios;
'z' sets step delay;
'F' triggers beeper;
'u' waits (value * 100 millisec);
'h' homes the system";
't' taring";
'T' Interactive calibration (NEW)");
'P' Calibrated weighting (NEW)");
