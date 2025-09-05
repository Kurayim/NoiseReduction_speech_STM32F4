# Low Pass Filter in STM32 & CMSIS-DSP

In the NoiseReduction_speech_STM32F4 repository, our main goal is to eliminate constant background noise from speech signals. However, at this stage—which is considered the initial phase of the project—our focus is on implementing a low-pass filter on the STM32F411 microcontroller using the CMSIS-DSP library.

The project scenario is defined as follows: a .wav audio file is stored on an SD card. The data from this file is then read, processed through the filter, and the filtered output is written back to the same SD card.

In this project, we use the STM32F411CEU6 microcontroller, which features a CORTEX-M4 processor, 512 KB of flash memory, and 128 KB of SRAM. One of the main challenges in this project is the limited SRAM, which does not allow the entire file to be read at once and processed through the filter in a single step.

To address this issue, the data is read from the SD card in frames of 1024 samples, with each sample represented as a 16-bit value. After processing and filtering each frame, the output is stored in a new file in the same format. This process is then repeated for subsequent frames until all the samples in the file have been processed.

It is worth noting that the data in the WAVE file is stored as 8-bit values, and for processing, two consecutive values must be combined to form a single 16-bit sample. Therefore, to create a frame of 1024 samples for processing, 2048 bytes must be read from the SD card. This process is handled by the WAV file-related functions implemented in the project.

Another important point is that, for filter implementation, it is preferable to change the data format by normalizing the sample values to the range [-1, 1]. This improves processing accuracy and filter performance and is also applied within the project’s WAV functions.



## نمونه کد
```c
#include <stdio.h>
int main() {
    printf("Hello, STM32!");
    return 0;
}
```


```c
#include <stdio.h>
int main() {
    printf("Hello, STM32!\n");
    return 0;
}
```

> این کد یک برنامه ساده به زبان C است که روی STM32 اجرا می‌شود:
```c
#include <stdio.h>
int main() {
    printf("Hello, STM32!\n");
    return 0;
}
```
![C Badge](https://img.shields.io/badge/language-C-blue)
```
