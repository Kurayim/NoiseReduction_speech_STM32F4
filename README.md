# Low Pass Filter in STM32 & CMSIS-DSP

In the NoiseReduction_speech_STM32F4 repository, our main goal is to eliminate constant background noise from speech signals. However, at this stage—which is considered the initial phase of the project—our focus is on implementing a low-pass filter on the STM32F411 microcontroller using the CMSIS-DSP library.

The project scenario is defined as follows: a .wav audio file is stored on an SD card. The data from this file is then read, processed through the filter, and the filtered output is written back to the same SD card.

In this project, we use the STM32F411CEU6 microcontroller, which features a CORTEX-M4 processor, 512 KB of flash memory, and 128 KB of SRAM. One of the main challenges in this project is the limited SRAM, which does not allow the entire file to be read at once and processed through the filter in a single step.

To address this issue, the data is read from the SD card in frames of 1024 samples, with each sample represented as a 16-bit value. After processing and filtering each frame, the output is stored in a new file in the same format. This process is then repeated for subsequent frames until all the samples in the file have been processed.
