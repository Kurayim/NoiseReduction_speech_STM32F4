# Low Pass Filter in STM32 & CMSIS-DSP

In the NoiseReduction_speech_STM32F4 repository, our main goal is to eliminate constant background noise from speech signals. However, at this stage—which is considered the initial phase of the project—our focus is on implementing a low-pass filter on the STM32F411 microcontroller using the CMSIS-DSP library.

The project scenario is defined as follows: a .wav audio file is stored on an SD card. The data from this file is then read, processed through the filter, and the filtered output is written back to the same SD card.

In this project, we use the STM32F411CEU6 microcontroller, which features a CORTEX-M4 processor, 512 KB of flash memory, and 128 KB of SRAM. One of the main challenges in this project is the limited SRAM, which does not allow the entire file to be read at once and processed through the filter in a single step.
To address this issue, the data is read from the SD card in frames of 1024 samples, with each sample represented as a 16-bit value. After processing and filtering each frame, the output is stored in a new file in the same format. This process is then repeated for subsequent frames until all the samples in the file have been processed.

It is worth noting that the data in the WAVE file is stored as 8-bit values, and for processing, two consecutive values must be combined to form a single 16-bit sample. Therefore, to create a frame of 1024 samples for processing, 2048 bytes must be read from the SD card. This process is handled by the WAV file-related functions implemented in the project.
Another important point is that, for filter implementation, it is preferable to change the data format by normalizing the sample values to the range [-1, 1]. This improves processing accuracy and filter performance and is also applied within the project’s WAV functions.

I will provide a set of explanations regarding the code of this program, which will contribute to a better understanding of the project.

>In this project, the implementation of the low-pass filter is carried out using two methods: FIR and IIR. By defining FILTER_M, the desired method can be selected and applied.
```c
// Select Low Pass Filter with FIR OR IIR
#define FIR_METHOD		1
#define IIR_METHOD		0
#define FILTER_M		FIR_METHOD
}
```


>The implementation of FIR and IIR methods requires the corresponding filter coefficients. In this project, the FIR low-pass filter coefficients are generated using the script Generate_FIR_LPF_Coeff.m, while the IIR low-pass filter coefficients are produced using Generate_IIR_LPF_Coeff.m. Both scripts are executable in MATLAB and provide the necessary coefficient values for implementing the filters on the microcontroller.
```c
%% FIR Low-pass Filter Coefficient Generator
clc; clear;

% ---- Filter Parameters ----
Fs = 22050;      % Sample Rate (Hz)
Fc = 1000;       % Cut-off Frequency (Hz)
N  = 128;        % Number of taps (filter order + 1)
windowType = 'hamming';  % Window type: 'hamming', 'blackman', or 'kaiser'
beta = 5;        % Only used for Kaiser window

% ---- Calculate Coefficients ----
Wn = Fc / (Fs/2);   % Normalize cutoff frequency to [0,1]

% Generate window array
switch lower(windowType)
    case 'hamming'
        win = hamming(N);
    case 'blackman'
        win = blackman(N);
    case 'kaiser'
        win = kaiser(N, beta);
    otherwise
        error('Window type not recognized');
end

b = fir1(N-1, Wn, win);  % Design filter using the window array

% ---- Normalization (optional) ----
b = b / sum(b);  % Ensure unity gain at DC

% ---- Display coefficients in the command window ----
fprintf('\n=== FIR Low-pass Filter Coefficients (%d taps) ===\n', N);
for i = 1:N
    fprintf('%3d: %+1.10f\n', i, b(i));
end

% ---- Display frequency response ----
figure;
freqz(b, 1, 1024, Fs);
title('Frequency Response of FIR Low-pass Filter');
grid on;

```

>>> این کد یک برنامه ساده به زبان C است که روی STM32 اجرا می‌شود:
```c
#include <stdio.h>
int main() {
    printf("Hello, STM32!\n");
    return 0;
}
```
![C Badge](https://img.shields.io/badge/language-C-blue)
```
