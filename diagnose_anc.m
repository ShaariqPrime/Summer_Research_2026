%% ANC Diagnostic Checklist
% Diagnostic file to indicate if system is working as intended
clear; clc;

%% Load the recorded data
fprintf('========== ANC DIAGNOSTICS ==========\n\n');

% Load files from indicated folder
try
    [mics, fs] = audioread('Recordings/mics_raw_debug_3.wav');
    x_noise = audioread('Recordings/x_noise_debug_3.wav');
    y_cancel = audioread('Recordings/y_cancel_debug_3.wav');
    fprintf('✓ Audio files loaded successfully\n');
catch
    error('Could not load audio files. Make sure they exist!');
end

%% Split into Phase 1 and Phase 2
half = floor(size(mics,1)/2);
phase1_err = mics(1:half, 1);           % Error mic baseline
phase2_err = mics(half+1:end, 1);       % Error mic with ANC
phase1_ref = mics(1:half, 2);           % Reference mic baseline
phase2_ref = mics(half+1:end, 2);       % Reference mic with ANC

x_phase1 = x_noise(1:half);
x_phase2 = x_noise(half+1:end);
y_phase1 = y_cancel(1:half);
y_phase2 = y_cancel(half+1:end);

%% Test 1: Is the noise actually reaching the error microphone?
fprintf('\n--- Test 1: Noise reaching error mic? ---\n');
rms_phase1 = sqrt(mean(phase1_err.^2));
rms_phase2 = sqrt(mean(phase2_err.^2));

fprintf('Phase 1 (baseline) RMS: %.6f (%.1f dBFS)\n', rms_phase1, 20*log10(rms_phase1+eps));
fprintf('Phase 2 (ANC) RMS:      %.6f (%.1f dBFS)\n', rms_phase2, 20*log10(rms_phase2+eps));

if rms_phase1 < 1e-4
    fprintf('❌ PROBLEM: Phase 1 signal too weak! Check:\n');
    fprintf('   - Is noise speaker working?\n');
    fprintf('   - Is err_chan mapped correctly?\n');
    fprintf('   - Increase noise amplitude (currently 0.03)\n');
else
    fprintf('✓ Noise is reaching error mic\n');
end

%% Test 2: Is there correlation between reference and error?
fprintf('\n--- Test 2: Correlation x vs error mic ---\n');
c1 = corr(x_phase1, phase1_err);
fprintf('Phase 1 correlation: %.3f\n', c1);

if abs(c1) < 0.2
    fprintf('❌ PROBLEM: Low correlation! This means:\n');
    fprintf('   - Noise speaker not creating measurable disturbance, OR\n');
    fprintf('   - Wrong microphone channel selected, OR\n');
    fprintf('   - Microphones too far from speakers\n');
elseif c1 < 0
    fprintf('⚠ WARNING: Negative correlation - might need to flip phase\n');
else
    fprintf('✓ Good correlation between noise and error mic\n');
end

%% Test 3: Is the control signal being generated?
fprintf('\n--- Test 3: Control signal generated? ---\n');
rms_y_phase1 = sqrt(mean(y_phase1.^2));
rms_y_phase2 = sqrt(mean(y_phase2.^2));

fprintf('Phase 1 control RMS: %.6f (should be ~0)\n', rms_y_phase1);
fprintf('Phase 2 control RMS: %.6f (should be >0)\n', rms_y_phase2);

if rms_y_phase2 < 1e-5
    fprintf('❌ PROBLEM: No control signal generated! Check:\n');
    fprintf('   - Are weights updating? (add print statements)\n');
    fprintf('   - Is mu too small? (try 1e-4 instead of 5e-6)\n');
    fprintf('   - Is secondary path estimate S_hat correct?\n');
else
    fprintf('✓ Control signal is being generated\n');
end

%% Test 4: Calculate actual reduction
fprintf('\n--- Test 4: Noise reduction achieved ---\n');
reduction_dB = 20*log10(rms_phase1/(rms_phase2+eps));
fprintf('Reduction: %.2f dB\n', reduction_dB);

if reduction_dB < 1
    fprintf('❌ PROBLEM: Less than 1 dB reduction!\n');
elseif reduction_dB < 5
    fprintf('⚠ WARNING: Only %.1f dB reduction (should be >5 dB)\n', reduction_dB);
else
    fprintf('✓ Good reduction: %.1f dB\n', reduction_dB);
end

%% Test 5: Spectral analysis at 400 Hz
fprintf('\n--- Test 5: Spectral analysis at 400 Hz ---\n');

% FFT of error mic signals
N = 2^16;
f = (0:N-1) * fs/N;
idx_400 = find(f >= 399 & f <= 401, 1);

% Phase 1
fft1 = abs(fft(phase1_err, N));
mag1_400 = fft1(idx_400);

% Phase 2
fft2 = abs(fft(phase2_err, N));
mag2_400 = fft2(idx_400);

fprintf('400 Hz magnitude Phase 1: %.6f\n', mag1_400);
fprintf('400 Hz magnitude Phase 2: %.6f\n', mag2_400);
fprintf('Reduction at 400 Hz: %.2f dB\n', 20*log10(mag1_400/(mag2_400+eps)));

%% Test 6: Check for clipping or saturation
fprintf('\n--- Test 6: Signal integrity ---\n');
if max(abs(phase1_err)) > 0.95
    fprintf('⚠ WARNING: Error mic signal clipping in Phase 1!\n');
end
if max(abs(phase2_err)) > 0.95
    fprintf('⚠ WARNING: Error mic signal clipping in Phase 2!\n');
end
if max(abs(y_phase2)) > 0.02
    fprintf('⚠ WARNING: Control signal hitting limiter \n');
    fprintf('   Consider increasing ymax or checking if weights are exploding\n');
end

%% Plot comparisons
figure('Name', 'ANC Performance Analysis');

% Time domain comparison
subplot(3,2,1);
t1 = (0:length(phase1_err)-1)/fs;
plot(t1, phase1_err);
xlabel('Time (s)'); ylabel('Amplitude');
title('Phase 1: Error Mic (Baseline)');
grid on;
ylim([-0.1 0.1]);

subplot(3,2,2);
t2 = (0:length(phase2_err)-1)/fs;
plot(t2, phase2_err);
xlabel('Time (s)'); ylabel('Amplitude');
title('Phase 2: Error Mic (ANC Active)');
grid on;
ylim([-0.1 0.1]);

% Spectra
subplot(3,2,3);
f_plot = f(1:N/2);
plot(f_plot, 20*log10(abs(fft1(1:N/2))+eps));
xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
title('Phase 1 Spectrum');
grid on;
xlim([0 1000]);

subplot(3,2,4);
plot(f_plot, 20*log10(abs(fft2(1:N/2))+eps));
xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
title('Phase 2 Spectrum');
grid on;
xlim([0 1000]);

% Control signal
subplot(3,2,5);
plot(t2, y_phase2);
xlabel('Time (s)'); ylabel('Amplitude');
title('Control Signal y(n)');
grid on;

% Overlay comparison
subplot(3,2,6);
plot(t1(1:min(fs,length(t1))), phase1_err(1:min(fs,length(t1)))); hold on;
plot(t2(1:min(fs,length(t2))), phase2_err(1:min(fs,length(t2))));
xlabel('Time (s)'); ylabel('Amplitude');
title('Comparison (First 1 second)');
legend('Baseline', 'ANC');
grid on;

%% Summary and recommendations
fprintf('\n========== SUMMARY ==========\n');
fprintf('Based on the tests above, here are likely issues:\n\n');

if rms_phase1 < 1e-4
    fprintf('1. CRITICAL: Increase noise amplitude or check speaker/mic connections\n');
end

if abs(c1) < 0.2
    fprintf('2. CRITICAL: Fix correlation issue - check microphone channels\n');
end

if rms_y_phase2 < 1e-5
    fprintf('3. CRITICAL: Control signal not generated - increase mu or check S_hat\n');
end

if reduction_dB < 1 && rms_phase1 > 1e-4 && rms_y_phase2 > 1e-5
    fprintf('4. Control signal exists but not canceling - possible issues:\n');
    fprintf('   a) Secondary path S_hat is incorrect or poorly aligned\n');
    fprintf('   b) Acoustic phase mismatch between noise and cancel speakers\n');
    fprintf('   c) Step size mu too small (try 1e-4 or larger)\n');
    fprintf('   d) Not enough adaptation time (increase phase duration)\n');
end

