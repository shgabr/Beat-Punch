## Team Members:
Mina Gamil

Salaheldin Soliman

Sherif Gabr

# Project Idea

This project is a clone of a famous product called [LiteBoxer](https://liteboxer.com/pages/homepage-2).

![](https://gaminghybrid.com/wp-content/uploads/2022/03/1647294943_959_Review-Liteboxer-VR-GMW3.jpg)

![](https://i.insider.com/600b52b47e47190011cb8dbd?width=1000&format=jpeg&auto=webp)


The goal is to create a smart, personal trainer that is able to progressively enhance the performance of the trainee, while keeping track of his workout statistics. The device will be able to keep track of calories burned, training hours, average response time, and other useful training metrics. The device will log all of the statisitics on the thingspeak platform and offer useful analytics. The user will be able to set a static pace for his training, or choose a song to train on its beats which is feature not offered on the current LiteBoxer product.

The challenge is analyzing the Fourier transform of the song in real-time while keeping track of the hits or misses of the user.


# Hardware Requirements

- STM32 Nucleo Board l432kc
- DFPlayer or SD card module
- WAVE Shield or AUX module
- External Speakers
- Bluetooth HC-05
- Wi-Fi Module ESP8266
- 6 Big Dome push buttons
- 2 meters LED strip
- LEDs
- Resistors
- Wires
- Bread Board


# Software Requirements

- STM32 CubeIDE
- FreeRTOS
- Audio Libraries
  - [OnsetsDS](http://onsetsds.sourceforge.net/)
  - [aubio](https://aubio.org)
  - [Beat and Tempo Tracking](https://github.com/michaelkrzyzaniak/Beat-and-Tempo-Tracking)

# Physical Connections

<img width="714" alt="Screen Shot 2022-04-22 at 11 32 09 PM" src="https://user-images.githubusercontent.com/50206867/164798658-b3747ae4-f93b-4727-80d7-28ed2c3e514d.png">

* The schematic uses an arduino as it was the only schematic available on circuit.io, but we will use the nucleo board.

[Circuit Diagram Link](https://www.circuito.io/app?components=512,11021,13678,336411,855863,855863,855863,855863,855863,855863,942700,942700,942700,942700,942700,942700)

# First Prototype

For the first prototype, we decided to implement the logic to light the LEDs at random for a predefined duration, which signals which button to press. Every LED is assigned a single button. When the user presses the correct button, a new LED will light at random and some statisitcal metrics are collected. If the user fails to press the correct button during the predefined period, then it will be counted as a missed punch, given maximum response rate. The metrics collected are response rate per punch, average response rate per round, and time took per round. They are communcaited to the user via USART.

## Prototype 1 Connections:
<img src="https://user-images.githubusercontent.com/50206867/166117711-19527b62-fdca-45c4-9d55-4369672cfb22.jpg" alt="Prototype 1 Connections" width="800"/>

### Hardware used in this prototype:
- STM32 l432kc nucleo board
- 6 push buttons
- 6 LEDs
- 6 1kΩ resistors
- Wires

## Demo:

![IMG_8059](https://user-images.githubusercontent.com/50206867/166117867-be31fe4a-1002-4f1a-b260-d45295a4be0e.GIF)



# Second Prototype
**In Progress**

# Final Prototype
 **In Progress**

# Limitations
**In Progress**

# References

[LiteBoxer Homepage](https://liteboxer.com/pages/homepage-2)
