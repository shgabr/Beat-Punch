# Repo:

[Github Repo](https://github.com/shgabr/Beat-Punch)

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

We can now mount an sd card and do the following operations on it:
- List all files of a certain extension on the file system
- Read and write to different files

We also configured the LCD screen to display messages to the user throughout the training, such as file names, training progress, and his performance.
Using the above file operations, we give the user the option to pick a certain music file from the sd card. This file is then preprocessed for its beats and stored in a memory buffer. Then, we start reading the wav file and playing it over the onboard Digital-to-Analog converter. We then start shuffling the LEDs based on the preprocessed beats while logging the user performance. Once the training is completed, the user has an option to restart training or to shutdown the session and save performance metrics to a file on the sd card.

Note: The beat analysis code works fine on Linux and Windows computers, but on the embedded device, it sometimes leads to runtime error due to unalloacted memory access. We are still trying to debug this to find the source of error. Also, the music playback works fine on its own, but when integrated with the complete code, the music player plays noise and not the real WAV file.

## Prototype 2 Connections:
![IMG_8128](https://user-images.githubusercontent.com/50206867/167273334-ccaba11e-f218-435a-961b-d89edf49acfd.JPG)

### Hardware used in this prototype:
- All prototype 1 hardware
- Micro SD card module
- AUX (TRRS) module
- LCD Screen
- Potentiometer 

\* Prototype 2 code is available on the [Github Repo](https://github.com/shgabr/Beat-Punch)

# Final Prototype

In the final prototype, we added a DFPlayer module after being unable to play good quality music using DAC and TRRS module. The DFPlayer reads the music files from an SD card and plays it over a 3W speaker. 

So, now the MicroSD card adapter is responsible for taking a specific format of the beats files *.txt, which is used to light the LEDs on each beat.

The user has the ability to choose which music file to play using 2 push buttons, and the beats file with the same name is chosen. 

Note: We changed the LCD to another nucleo board due to the limitations of the pins. Now, we have 2 nucleo boards which communicate over UART. One is responsible for the training playing music, and applying beats, and the other is responsible for displaying the message recieved from the UART over the LCD.

## Final Prototype Connections:

![File_000](https://user-images.githubusercontent.com/50206867/169003816-58fc8769-7580-41f2-a1b0-a6ffccd2ed38.png)


### The Final Hardware used in the project:
- 2 STM32 l432 nucleo boards
- 3 push buttons
- 3 LEDs
- 3 1kΩ resistors
- Micro SD card module
- DFPlayer
- 2 microSD
- 3W Speaker
- LCD Screen
- Potentiometer 
- Wires & Jumpers

[Demo Link](https://youtu.be/O_sArSq9wAU)

\* Final Prototype code is available on the [Github Repo](https://github.com/shgabr/Beat-Punch) inside the "finalPrototype" folder

# Limitations
- LCD communication can be unreliable

# References

[LiteBoxer Homepage](https://liteboxer.com/pages/homepage-2)
