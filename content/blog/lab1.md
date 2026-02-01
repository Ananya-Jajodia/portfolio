+++
title = "Lab 1: Artemis and Bluetooth"
description = "This lab introduces programming the RedBoard Artemis Nano and explores communicating with the board via Bluetooth"
date = "2026-01-23"

[taxonomies]
tags = ["ece5160", "Artemis", "C Programming", "Bluetooth"]
+++

<!-- This is your first blog post! 
You can:

* Write in *Markdown*
* Add code blocks: -->

This lab introduces programming the RedBoard Artemis Nano and explores communicating with the board via Bluetooth.  

## Lab 1A
The first part of this lab demostrates how to program the RedBoard Artemis Nano and explores some of the features via demo code files. 

### Basic Setup



### Running Blink
Like with most microntroller projects, we start with making the onboard led blink. The demo code initializes the built-in led as the output, then repeated turns it on and off with a second delay in between each action.

[TODO: Embed video](assets\lab1\1a_task2.mov)



### Running Serial
Next, we run the Serial demo code to demonstate communication with the Artemis via the serial monitor. The serial monitor allows us to send and recieve messages when the Artemis is connected to a serial port on our laptop. The demo code takes a message you sent and echos it back as demontrated in the video below. 

TODO: Embed video

### Testing the Temperature Sensor
The "analog read" demo code allows us to read the output of the Artemis builtin temperature sensor. 

TODO: mayeb add to this and embed pic


### Testing the Microphone
Finally, the Microphone Ouptut demo allows use to see the loudest frequency detected by the onboard microphone. You can see the result of my high pitch whisper screaming into the microphone below. 

TODO: Add the screenshot



#### Grad Task: Frequency Detector
The grad task for this assignment builds on the Microphone and Serial demos to create a electronic tuner that can detect 3 different frequencies. I chose to detect the musical notes A, C and F. I did this by playing the middle pitches of these notes (A4, C4, and F4) and setting the standard to the output of the program. To detect the pitch, I tranform the heard pitch to be in the correct octave (by multiplying or dividing by 2), then find the pitch it is closest to and print that as the note. 

```c
void pitch_analysis(uint32_t real_pitch) {

  // standard frequencies to match 
  uint32_t freq_A = 434;
  uint32_t freq_C = 263;
  uint32_t freq_F = 343;

  uint32_t pitch = real_pitch;

  // We normalize the pitch to be in the middle octave
  while (pitch < 256) {
    pitch *= 2;
  }

  while (pitch > 512){
    pitch /= 2;
  }

  uint32_t diff_A = (uint32_t) abs((int) pitch - (int) freq_A);
  uint32_t diff_C = (uint32_t) abs((int) pitch - (int) freq_C);
  uint32_t diff_F = (uint32_t) abs((int) pitch - (int) freq_F);

  // We choose the closest pitch and print it
  if (diff_A < diff_C) {
    if (diff_F < diff_A) {
      Serial.printf("THE NOTE IS AN F (%d)         \n", real_pitch);
    }
    else {
      Serial.printf("THE NOTE IS AN A (%d)         \n", real_pitch);
    }
  }
  else if (diff_F < diff_C) {
    Serial.printf("THE NOTE IS AN F (%d)         \n", real_pitch);
  }  
  else {
    Serial.printf("THE NOTE IS A C (%d)         \n", real_pitch);
  }
}
```

TODO: add video


<!-- * Create [links](https://example.com)
* Add images
* And much more!

Check [Linkita's documentation](https://codeberg.org/salif/linkita) to learn about all features. -->


## Lab 1B
The second part of this lab works on communicated with the Artemis via bluetooth. Since we won't have a serial connection with the board when it is in a robot, our primary communication method will be Bluetooth. In this lab, we experiement with thie bluetooth connection and create commands to use for communication.

### Prelab
Setup: Briefly describe the steps taken to set up your computer for Lab 1, showing any results (i.e. MAC address printing)

Add a brief explanation of your understanding of the codebase and how Bluetooth works between your computer and the Artemis


### ECHO Command

### SEND_THREE_FLOATS command 

### GET_TIME_MILLIS command

### Notification Handler

### Sending Time in a Loop

### SEND_TIME_DATA (Sending Time in and Array)

### GET_TEMP_READINGS (Sending Temperature)

### Discussion
Discuss the differences between these two methods, the advantages and disadvantages of both and the potential scenarios that you might choose one method over the other. How “quickly” can the second method record data? The Artemis board has 384 kB of RAM. Approximately how much data can you store to send without running out of memory?


### Effective Data Rate, Overhead, and Reliablility
Do many short packets introduce a lot of overhead? Do larger replies help to reduce overhead? 