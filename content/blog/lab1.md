+++
title = "Lab 1: Artemis and Bluetooth"
description = "This lab introduces programming the RedBoard Artemis Nano and explores communicating with the board via Bluetooth"
date = "2026-01-23"

[taxonomies]
tags = ["ece5160", "Artemis", "C Programming", "Bluetooth"]
+++

This is your first blog post! <!-- more -->
You can:

* Write in *Markdown*
* Add code blocks:

## Lab 1A


### Basic Setup

### Running Blink

### Running Serial

### Testing the Temperature Sensor

### Testing the Microphone

#### Grad Task: Frequency Detector

```c
void pitch_analysis(uint32_t real_pitch) {
  uint32_t freq_A = 434;
  uint32_t freq_C = 263;
  uint32_t freq_F = 343;

  uint32_t pitch = real_pitch;

  while (pitch < 256) {
    pitch *= 2;
  }

  while (pitch > 512){
    pitch /= 2;
  }

  uint32_t diff_A = (uint32_t) abs((int) pitch - (int) freq_A);
  uint32_t diff_C = (uint32_t) abs((int) pitch - (int) freq_C);
  uint32_t diff_F = (uint32_t) abs((int) pitch - (int) freq_F);




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

* Create [links](https://example.com)
* Add images
* And much more!

Check [Linkita's documentation](https://codeberg.org/salif/linkita) to learn about all features.

## Lab 1B