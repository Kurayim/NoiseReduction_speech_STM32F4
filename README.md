# Low Pass Filter in STM32 & CMSIS-DSP

In the NoiseReduction_speech_STM32F4 repository, our main goal is to eliminate constant background noise from speech signals. However, at this stage—which is considered the initial phase of the project—our focus is on implementing a low-pass filter on the STM32F411 microcontroller using the CMSIS-DSP library.

The project scenario is defined as follows: a .wav audio file is stored on an SD card. The data from this file is then read, processed through the filter, and the filtered output is written back to the same SD card.
