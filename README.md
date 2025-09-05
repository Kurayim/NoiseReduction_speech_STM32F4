# Low Pass Filter in STM32 & CMSIS-DSP

In the NoiseReduction_speech_STM32F4 repository, our main goal is to eliminate constant background noise from speech signals. However, at this stage—which is considered the initial phase of the project—our focus is on implementing a low-pass filter on the STM32F411 microcontroller using the CMSIS-DSP library.

The project scenario is defined as follows: a .wav audio file is stored on an SD card. The data from this file is then read, processed through the filter, and the filtered output is written back to the same SD card.

In this project, we use the STM32F411CEU6 microcontroller, which features a CORTEX-M4 processor, 512 KB of flash memory, and 128 KB of SRAM. One of the main challenges in this project is the limited SRAM, which does not allow the entire file to be read at once and processed through the filter in a single step.
To address this issue, the data is read from the SD card in frames of 1024 samples, with each sample represented as a 16-bit value. After processing and filtering each frame, the output is stored in a new file in the same format. This process is then repeated for subsequent frames until all the samples in the file have been processed.

It is worth noting that the data in the WAVE file is stored as 8-bit values, and for processing, two consecutive values must be combined to form a single 16-bit sample. Therefore, to create a frame of 1024 samples for processing, 2048 bytes must be read from the SD card. This process is handled by the WAV file-related functions implemented in the project.
Another important point is that, for filter implementation, it is preferable to change the data format by normalizing the sample values to the range [-1, 1]. This improves processing accuracy and filter performance and is also applied within the project’s WAV functions.

I will provide a set of explanations regarding the code of this program, which will contribute to a better understanding of the project.
<br></br>



>In this project, the implementation of the low-pass filter is carried out using two methods: FIR and IIR. By defining FILTER_M, the desired method can be selected and applied.<
```c
// Select Low Pass Filter with FIR OR IIR
#define FIR_METHOD		1
#define IIR_METHOD		0
#define FILTER_M		FIR_METHOD
}
```


        
The implementation of FIR and IIR methods requires the corresponding filter coefficients. In this project, the FIR low-pass filter coefficients are generated using the script Generate_FIR_LPF_Coeff.m, while the IIR low-pass filter coefficients are produced using Generate_IIR_LPF_Coeff.m. Both scripts are executable in MATLAB and provide the necessary coefficient values for implementing the filters on the microcontroller.



    

#Generat_FIR_LPF_Coeff
This program is designed for generating FIR filter coefficients. By appropriately setting the parameters Fs (sampling frequency), Fc (cutoff frequency), N (number of taps), and windowType (type of window), and, if necessary, beta (for the Kaiser window), the required filter coefficients can be obtained from the program’s output.
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
%% IIR Low-pass Butterworth Coefficients Printer (b,a + SOS + CMSIS)
clc; clear;

% ---- Filter specs ----
fs    = 22050;     % Sample rate (Hz)
fc    = 2000;      % Cutoff frequency (Hz)
order = 6;         % Butterworth order (even -> 3 biquads)

Wn = fc/(fs/2);    % Normalize cutoff to Nyquist

% ---- Direct-form (b,a) coefficients ----
% That is, what percentage of the frequency does Wn pass between 0 and (Fs/2)?
[b, a] = butter(order, Wn, 'low');   % digital IIR Butterworth (bilinear)

% Optional: ensure unity DC gain (usually already ~1 for lowpass Butter)
% gdc = sum(b)/sum(a);  b = b/gdc;

% ---- Pretty print (b,a) ----
fprintf('\n=== Butterworth Low-pass (order=%d, fc=%g Hz, fs=%g Hz) ===\n', order, fc, fs);
fprintf('--- Direct-Form Coefficients (b) ---\n');
for i = 1:numel(b)
    fprintf('b(%d) = %+1.15f\n', i, b(i));
end

fprintf('\n--- Direct-Form Coefficients (a) ---\n');
for i = 1:numel(a)
    fprintf('a(%d) = %+1.15f\n', i, a(i));
end

% ---- SOS (biquads) for stable implementation ----
% Get zeros, poles, gain then convert to SOS
[z, p, k] = butter(order, Wn, 'low');
[sos, g]  = zp2sos(z, p, k);     % sos: [b0 b1 b2 a0 a1 a2], gain g

% Apply the overall gain g to the FIRST section numerator
sos(1,1:3) = sos(1,1:3) * g;

fprintf('\n--- SOS (biquads) [b0 b1 b2 a0 a1 a2], gain applied to first section ---\n');
for s = 1:size(sos,1)
    fprintf('SOS%u: ', s);
    fprintf('%+1.15f ', sos(s,:));
    fprintf('\n');
end

% ---- CMSIS-DSP biquad coeffs (DF1/DF2): {b0, b1, b2, a1, a2} per section ----
% Note: CMSIS expects a1,a2 with SIGN NEGATED (difference eqn has minus signs).
numStages = size(sos,1);
cmsisCoeffs = zeros(numStages, 5);

for s = 1:numStages
    b0 = sos(s,1); b1 = sos(s,2); b2 = sos(s,3);
    a0 = sos(s,4); a1 = sos(s,5); a2 = sos(s,6);

    % Normalize to a0 = 1 (safety; usually already 1)
    b0 = b0/a0; b1 = b1/a0; b2 = b2/a0; a1 = a1/a0; a2 = a2/a0;

    % CMSIS format: {b0, b1, b2, -a1, -a2}
    cmsisCoeffs(s,:) = [b0, b1, b2, -a1, -a2];
end

fprintf('\n--- CMSIS-DSP biquad coeffs per stage: {b0, b1, b2, a1, a2} with a1,a2 NEGATED ---\n');
for s = 1:numStages
    fprintf('Stage %d: { %+1.15ff, %+1.15ff, %+1.15ff, %+1.15ff, %+1.15ff }\n', ...
        s, cmsisCoeffs(s,1), cmsisCoeffs(s,2), cmsisCoeffs(s,3), cmsisCoeffs(s,4), cmsisCoeffs(s,5));
end

% ---- (Optional) Quick frequency response check in MATLAB (not required) ----
% figure; freqz(b,a, 2048, fs); grid on; title('Butterworth LPF freq response');

```
![C Badge](https://img.shields.io/badge/language-C-blue)
```
