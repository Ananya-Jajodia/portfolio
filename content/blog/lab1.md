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

<video width="640" height="480" controls>
  <source src="assets/lab1/1a_task2.mov" type="video/mov">
  Your browser does not support the video tag.
</video>



### Running Serial
Next, we run the Serial demo code to demonstate communication with the Artemis via the serial port. The serial monitor allows us to send and recieve messages when the Artemis is connected to a serial port on our laptop. The demo code takes a message you sent and echos it back as demontrated in the video below. 

TODO: Embed video

### Testing the Temperature Sensor
The "analog read" demo code allows us to read the output of the Artemis builtin temperature sensor. 

TODO: mayeb add to this and embed pic


### Testing the Microphone
Finally, the Microphone Ouptut demo allows use to see the loudest frequency detected by the onboard microphone. You can see the result of my high pitch whisper screaming into the microphone below. 

TODO: Add the screenshot



### Grad Task: Frequency Detector
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
TODO: Finish this
Setup: Briefly describe the steps taken to set up your computer for Lab 1, showing any results (i.e. MAC address printing)

Add a brief explanation of your understanding of the codebase and how Bluetooth works between your computer and the Artemis


### ECHO Command
The `ECHO` command takes in a string `s` and prints "Robot says: `s` :)" on the serial port.


``` c
case ECHO:
    char char_arr[MAX_MSG_SIZE];

    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(char_arr);
    if (!success)
        return;

    Serial.print("Robot says: ");
    Serial.print(char_arr);
    Serial.println(" :)");
    
    break;
```

![result of running commands in serial monitor](assets\lab1\1b_task2.png)


### SEND_THREE_FLOATS command 
The `SEND_THREE_FLOATS` command takes in an input of 3 floats each separated by a `|`. The robot prints to the serial port `Three floats: [float1], [float2], [float3]`.

``` c
case SEND_THREE_FLOATS:
    float float_a, float_b, float_c;

    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(float_a);
    if (!success)
        return;

    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(float_b);
    if (!success)
        return;

    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(float_c);
    if (!success)
        return;

    Serial.print("Three Floats: ");
    Serial.print(float_a);
    Serial.print(", ");
    Serial.print(float_b);
    Serial.print(", ");
    Serial.println(float_c);
    
    break;
```


### GET_TIME_MILLIS command
The `GET_TIME MILLIS` command takes in no parameters, gets the time since Artemis boot in milliseconds, and writes `T: [time]` to the string characteristic and the serial port.

``` c
case GET_TIME_MILLIS:
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append((int) millis());
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.println(tx_estring_value.c_str());
    break;
```

<video width="320" height="240" controls>
  <source src="\assets\lab1\1b_task3.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>


### Notification Handler
The notification handler takes in a function with the UUID and a bytearray as inputs. It is set up to call the function when the string characteristic is written. This notification handler waits for the time to be written, parsed the data, and prints it.

``` python
def time_extractor(uuid, byte_time):
    str_time = ble.bytearray_to_string(byte_time)
    i = str_time.find("T:")
    if i  == -1:
        print(f"Time is {str_time}")
    else:
        print(f"Time is {str_time[i+2:]}") 

ble.start_notify(ble.uuid['RX_STRING'], time_extractor)
```

<video width="320" height="240" controls>
  <source src="\assets\lab1\1b_task4.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>


### Sending Time in a Loop
We start by sending time in a loop. I do this by making a command that gets the time and transmits it in a loop. From there, the notification handler parses and prints the data.

``` c
case LOOP_TIME:
    Serial.println("Sending Loop Time");
    for (i = 0; i < TIME_ARR_SIZE; i++) {
        tx_estring_value.clear();
        tx_estring_value.append((int) millis());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    Serial.println("Loop Complete");
    break;
```

<video width="320" height="240" controls>
  <source src="\assets\lab1\1b_task5.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>



### SEND_TIME_DATA (Sending Time in and Array)
Next, we transmit the data by storing the values in an array, then transmitting all at once. 

``` c
case SEND_TIME_DATA:
    Serial.println("Filling Time");
    for (i = 0; i < TIME_ARR_SIZE; i++) {
        time_stamps[i] = (int) millis();
    }            
    Serial.println("Time Array Complete");
    Serial.println("Sending Array Time");
    for (i = 0; i < TIME_ARR_SIZE; i++) {
        tx_estring_value.clear();
        tx_estring_value.append(time_stamps[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }            
    Serial.println("Array Complete");
    break;
```

<video width="320" height="240" controls>
  <source src="\assets\lab1\1b_task6.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>


### GET_TEMP_READINGS (Sending Temperature)
Similar to `SEND_TIME_DATA`, we fill 2 arrays with data for the time stamp and the current temperature reading. Then, we transmit the data.

``` c
case GET_TEMP_READINGS:
    Serial.println("Filling Readings");
    for (i = 0; i < TIME_ARR_SIZE; i++) {
        float temp_f = getTempDegF();
        time_stamps[i] = (int) millis();
        temp_readings[i] = getTempDegF();
    }            
    Serial.println("Time Array Complete");
    Serial.println("Sending Array Time");
    for (i = 0; i < TIME_ARR_SIZE; i++) {
        tx_estring_value.clear();
        tx_estring_value.append(time_stamps[i]);
        tx_estring_value.append(":");
        tx_estring_value.append(temp_readings[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }            
    Serial.println("Array Complete");
    break;
```

<video width="320" height="240" controls>
  <source src="\assets\lab1\1b_task7.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>




### Discussion
#### Loop vs Array

We can clearly see that the time stamps are much closer together when the data is first stored in array, and then transmitted, like in `SEND_TIME_DATA`. The `LOOP_TIME` transmission shows a delay of about 66.6 ms. Meanwhile, all 10 datapoints from `SEND_TIME_DATA` and in the same millisecond. This shows that if we want to collect data quickly, we should be storing it in an array. But, if we want to transmit the most up-to-date data, then sending data in a loop makes more sense. 

#### Amount of Data

We transmit time as an `int` which is 4B. If all the Artemis RAM is used to store time, we could store at most 96000 time values. This is assuming no other data is in RAM and no other overhead is needed be to stored. 


### Effective Data Rate, Overhead, and Reliablility
To observe the data rate of messages of different sizes, I observe the time between transmitting and recieving a string of various lengths. Since each character is 1 byte, the string length is the same as the byte size of the message. I form the string first, record the start time, then transmit. I tested 100 transmission of each byte size and then graphed the average time.

![Graph of Round Trip Time vs Data Length](assets\lab1\grad_task_1b.png)

From this graph, we see that smallest and largest packet size show a similar transmission time. This indicates the small packets do introduce a lot of overhead. We see the round trip time spike when the data packets are 100-120 bytes. Overall, the data rate doesn't seem to change too much when the data size changes, showing that there is limited overhead on the messages in general.

Even as the data rate changed, no packets were dropped at any point. This shows that the computer does read all the data from the Artemis. 

#### Implementation
This command was used to send back the recieved data.
```c
case DATA_RATE_ANALYSIS:
    char message_arr[MAX_MSG_SIZE];
    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(message_arr);
    if (!success)
        return;
    
    tx_estring_value.clear();
    tx_estring_value.append(message_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
```

On the python side, we choose to take 100 samples of strings from 1 byte to 140 bytes. 

``` python
max_byte_size = 141
samples = 100

start_times = [[] for _ in range(max_byte_size)]
end_times = [[] for _ in range(max_byte_size)]
```

A notification handler is set up to recieve messages and add the end time to the appropiate array spot.

``` python 
def recieved_time_getter(uuid, byte_message):    
    end_time = time.time()
    mess = ble.bytearray_to_string(byte_message)
    end_times[len(mess)].append(end_time)
 
        
ble.start_notify(ble.uuid['RX_STRING'], recieved_time_getter)
```

Then, we transmit messages of various sizes. We record the time they're transmitted and sleep to ensure transmissions don't conflict with eachother.

``` python
for i in range(1, max_byte_size-1):
    for _ in range(samples):
        message = "a"*i
        start = time.time()
        ble.send_command(CMD.DATA_RATE_ANALYSIS, message)
        start_times[i].append(start)
        time.sleep(.05)
print("done")
```

Finally, we compute the rtt and graph the mean.

``` python
ble.stop_notify(ble.uuid['RX_STRING'])

start = np.array(start_times[1:max_byte_size-1])
end = np.array(end_times[1:max_byte_size-1])

durations = end - start
rtt = durations.mean(axis=1)

plt.plot(rtt)
plt.xlabel("Message Size (bytes)")
plt.ylabel("Round Trip Time (ms)")
plt.show()
```