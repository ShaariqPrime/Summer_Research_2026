%% PROFESSIONAL ANC PRESENTATION GRAPHICS
% Creates publication-quality figures for presenting ANC system results
% Run this after your ANC experiment to generate clean visualizations

clear; clc; close all;

%% ========== USER CONFIGURATION ==========
target_freq = 500;  % Target frequency in Hz
filename_suffix = '';  % Suffix for audio files

%% Load recorded data
[mics, fs] = audioread(['mics_raw' filename_suffix '.wav']);
x_noise = audioread(['ref_noise' filename_suffix '.wav']);
y_cancel = audioread(['y_cancel' filename_suffix '.wav']);

% Split into phases
half = floor(size(mics,1)/2);
phase1_err = mics(1:half, 1);
phase2_err = mics(half+1:end, 1);
phase1_ref = mics(1:half, 2);
phase2_ref = mics(half+1:end, 2);
y_control = y_cancel(half+1:end);

%% Calculate key metrics
rms1 = sqrt(mean(phase1_err.^2));
rms2 = sqrt(mean(phase2_err.^2));
reduction_dB = 20*log10(rms1/(rms2+eps));

%% FIGURE 1: Time Domain Comparison
fig1 = figure('Position', [100 100 1600 900], 'Color', 'w');

% Baseline waveform
subplot(3,2,1);
t1 = (0:length(phase1_err)-1)/fs;
plot(t1, phase1_err(1:length(t1)), 'b', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Phase 1: Baseline (No ANC)', 'FontSize', 13, 'FontWeight', 'bold');
ylim([-0.15 0.15]);

% ANC active waveform
subplot(3,2,2);
t2 = (0:length(phase2_err)-1)/fs;
plot(t2, phase2_err(1:length(t2)), 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Phase 2: ANC Active', 'FontSize', 13, 'FontWeight', 'bold');
ylim([-0.15 0.15]);

% Zoomed comparison (1 second)
subplot(3,2,3);
t_zoom = (0:fs-1)/fs;
plot(t_zoom, phase1_err(1:fs), 'b', 'LineWidth', 1.5); hold on;
plot(t_zoom, phase2_err(1:fs), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Detailed Comparison (First 1 Second)', 'FontSize', 13, 'FontWeight', 'bold');
legend('Baseline', 'ANC Active', 'Location', 'best', 'FontSize', 10);

% Control signal
subplot(3,2,4);
plot(t2, y_control(1:length(t2)), 'Color', [0 0.6 0], 'LineWidth', 1.2);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
title('Control Signal y(n)', 'FontSize', 13, 'FontWeight', 'bold');

% RMS over time
subplot(3,2,5);
win = fs;  % 1 second windows
rms_baseline = zeros(floor(length(phase1_err)/win), 1);
rms_anc = zeros(floor(length(phase2_err)/win), 1);
for i = 1:length(rms_baseline)
    rms_baseline(i) = sqrt(mean(phase1_err((i-1)*win+1:i*win).^2));
    rms_anc(i) = sqrt(mean(phase2_err((i-1)*win+1:i*win).^2));
end
t_rms = (0:length(rms_baseline)-1);
plot(t_rms, 20*log10(rms_baseline+eps), 'b-o', 'LineWidth', 2, 'MarkerSize', 6); hold on;
plot(t_rms, 20*log10(rms_anc+eps), 'r-s', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
xlabel('Time Window (seconds)', 'FontSize', 11);
ylabel('RMS Level (dBFS)', 'FontSize', 11);
title('RMS Level Over Time', 'FontSize', 13, 'FontWeight', 'bold');
legend('Baseline', 'ANC', 'Location', 'best', 'FontSize', 10);

% Performance summary
subplot(3,2,6);
axis off;
summary_text = {
    '\bf Performance Summary', 
    '', 
    sprintf('Baseline RMS: %.6f (%.1f dBFS)', rms1, 20*log10(rms1+eps)),
    sprintf('ANC RMS: %.6f (%.1f dBFS)', rms2, 20*log10(rms2+eps)),
    '',
    sprintf('\\bf\\color{blue}Noise Reduction: %.2f dB', reduction_dB),
    '',
    'Algorithm: FxNLMS',
    sprintf('Target Frequency: %d Hz', target_freq),
    sprintf('Adaptation Time: %d seconds', round(length(phase2_err)/fs))
};
text(0.1, 0.5, summary_text, 'FontSize', 12, 'VerticalAlignment', 'middle');

%% FIGURE 2: Frequency Domain Analysis
fig2 = figure('Position', [100 100 1600 800], 'Color', 'w');

% FFT parameters
N = 2^17;
f = (0:N/2-1)*fs/N;
fft1 = abs(fft(phase1_err, N));
fft2 = abs(fft(phase2_err, N));

% Full spectrum comparison
subplot(2,2,1);
max_freq_plot = min(5000, fs/2);  % Plot up to 5 kHz or Nyquist
plot(f, 20*log10(fft1(1:N/2)+eps), 'b', 'LineWidth', 1.5); hold on;
plot(f, 20*log10(fft2(1:N/2)+eps), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Magnitude (dB)', 'FontSize', 11);
title(sprintf('Full Spectrum: 0-%d Hz', max_freq_plot), 'FontSize', 13, 'FontWeight', 'bold');
xlim([0 max_freq_plot]);
legend('Baseline', 'ANC', 'FontSize', 10);

% Zoomed around target frequency
subplot(2,2,2);
freq_range = [target_freq-200, target_freq+200];
idx_zoom = find(f >= freq_range(1) & f <= freq_range(2));
plot(f(idx_zoom), 20*log10(fft1(idx_zoom)+eps), 'b', 'LineWidth', 2); hold on;
plot(f(idx_zoom), 20*log10(fft2(idx_zoom)+eps), 'r', 'LineWidth', 2);
xline(target_freq, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Magnitude (dB)', 'FontSize', 11);
title(sprintf('Zoomed: %d-%d Hz (Target: %d Hz)', freq_range(1), freq_range(2), target_freq), ...
    'FontSize', 13, 'FontWeight', 'bold');
legend('Baseline', 'ANC', sprintf('Target (%d Hz)', target_freq), 'FontSize', 10);

% Reduction spectrum
subplot(2,2,3);
reduction_spectrum = 20*log10(fft1(1:N/2)./(fft2(1:N/2)+eps));
plot(f, reduction_spectrum, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5);
xline(target_freq, 'k--', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Reduction (dB)', 'FontSize', 11);
title('Noise Reduction vs Frequency', 'FontSize', 13, 'FontWeight', 'bold');
xlim([0 max_freq_plot]);

% Bar chart at key frequencies
subplot(2,2,4);
% Create frequency points around target
freqs_of_interest = [target_freq/4, target_freq/2, target_freq, target_freq*1.5, target_freq*2];
freqs_of_interest = round(freqs_of_interest);
baseline_vals = zeros(size(freqs_of_interest));
anc_vals = zeros(size(freqs_of_interest));
for i = 1:length(freqs_of_interest)
    [~, idx] = min(abs(f - freqs_of_interest(i)));
    baseline_vals(i) = 20*log10(fft1(idx)+eps);
    anc_vals(i) = 20*log10(fft2(idx)+eps);
end
x_pos = 1:length(freqs_of_interest);
bar_width = 0.35;
bar(x_pos - bar_width/2, baseline_vals, bar_width, 'b'); hold on;
bar(x_pos + bar_width/2, anc_vals, bar_width, 'r');
set(gca, 'XTick', x_pos, 'XTickLabel', freqs_of_interest);
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Magnitude (dB)', 'FontSize', 11);
title('Magnitude at Key Frequencies', 'FontSize', 13, 'FontWeight', 'bold');
legend('Baseline', 'ANC', 'FontSize', 10);
grid on;

%% FIGURE 3: Spectrogram Comparison
fig3 = figure('Position', [100 100 1600 600], 'Color', 'w');

window = hamming(1024);
noverlap = 512;
nfft = 2048;

% Baseline spectrogram
subplot(1,2,1);
spectrogram(phase1_err, window, noverlap, nfft, fs, 'yaxis');
ylim([0 1]);
title('Phase 1: Baseline Spectrogram', 'FontSize', 13, 'FontWeight', 'bold');
colormap('turbo'); colorbar; clim([-100 -50]);

% ANC spectrogram
subplot(1,2,2);
spectrogram(phase2_err, window, noverlap, nfft, fs, 'yaxis');
ylim([0 1]);
title('Phase 2: ANC Active Spectrogram', 'FontSize', 13, 'FontWeight', 'bold');
colormap('turbo'); colorbar; clim([-100 -50]);

%% FIGURE 4: Performance Metrics Dashboard
fig4 = figure('Position', [100 100 1400 800], 'Color', 'w');

% Big reduction number
subplot(2,3,[1 2]);
axis off;
if reduction_dB > 0
    color_val = [0 0.7 0];
    status = 'SUCCESS';
else
    color_val = [0.8 0.2 0];
    status = 'REINFORCEMENT';
end
text(0.5, 0.7, sprintf('%.1f dB', abs(reduction_dB)), ...
    'FontSize', 80, 'FontWeight', 'bold', 'Color', color_val, ...
    'HorizontalAlignment', 'center');
text(0.5, 0.4, 'Noise Reduction', 'FontSize', 24, ...
    'HorizontalAlignment', 'center');
text(0.5, 0.2, status, 'FontSize', 20, 'Color', color_val, ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold');

% RMS levels
subplot(2,3,3);
b = bar([1 2], [20*log10(rms1+eps), 20*log10(rms2+eps)]);
b.FaceColor = 'flat';
b.CData(1,:) = [0.3 0.3 0.8];
b.CData(2,:) = [0.8 0.3 0.3];
set(gca, 'XTickLabel', {'Baseline', 'ANC'});
ylabel('RMS Level (dBFS)', 'FontSize', 12);
title('Average RMS Comparison', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
for i = 1:2
    text(i, b.YData(i), sprintf('%.1f dB', b.YData(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 11, 'FontWeight', 'bold');
end

% Target frequency specific
subplot(2,3,4);
[~, idx_target] = min(abs(f - target_freq));
mag_target_baseline = 20*log10(fft1(idx_target)+eps);
mag_target_anc = 20*log10(fft2(idx_target)+eps);
reduction_target = mag_target_baseline - mag_target_anc;
b2 = bar([1 2], [mag_target_baseline, mag_target_anc]);
b2.FaceColor = 'flat';
b2.CData(1,:) = [0.3 0.3 0.8];
b2.CData(2,:) = [0.8 0.3 0.3];
set(gca, 'XTickLabel', {'Baseline', 'ANC'});
ylabel(sprintf('Magnitude at %d Hz (dB)', target_freq), 'FontSize', 12);
title(sprintf('Target Frequency (%d Hz)', target_freq), 'FontSize', 13, 'FontWeight', 'bold');
grid on;
text(1.5, mean([mag_target_baseline, mag_target_anc]), ...
    sprintf('Δ = %.1f dB', reduction_target), ...
    'HorizontalAlignment', 'center', 'FontSize', 12, ...
    'FontWeight', 'bold', 'BackgroundColor', 'w');

% System info
subplot(2,3,[5 6]);
axis off;
info_text = {
    '\bf System Configuration',
    '',
    'Hardware:',
    '  • Behringer Audio Interface (4 channels)',
    '  • Sony Speaker Array (8 channels)',
    '  • Reference Mic: Channel 2',
    '  • Error Mic: Channel 1',
    '  • Cancel Speaker: Channel 7',
    '',
    'Algorithm Parameters:',
    sprintf('  • Sample Rate: %d Hz', fs),
    '  • Filter Length (Lw): Adaptive FIR',
    '  • Step Size (μ): Optimized NLMS',
    sprintf('  • Frame Size: %d samples', 512),
    '  • Adaptation: FxNLMS with leakage',
    '',
    'Test Conditions:',
    sprintf('  • Target Frequency: %d Hz', target_freq),
    sprintf('  • Test Duration: %d seconds', round(size(mics,1)/fs))
};
text(0.05, 0.95, info_text, 'FontSize', 10, ...
    'VerticalAlignment', 'top', 'FontName', 'Courier');

%% Save all figures
saveas(fig1, 'ANC_1_Time_Domain_Analysis.png');
saveas(fig2, 'ANC_2_Frequency_Domain_Analysis.png');
saveas(fig3, 'ANC_3_Spectrogram_Comparison.png');
saveas(fig4, 'ANC_4_Performance_Dashboard.png');

fprintf('\n✓ Generated 4 presentation figures\n');
fprintf('  Files saved as PNG in current directory\n\n');
fprintf('Summary for presentation:\n');
fprintf('  Target Frequency: %d Hz\n', target_freq);
fprintf('  Overall Noise Reduction: %.2f dB\n', reduction_dB);
fprintf('  Reduction at %d Hz: %.2f dB\n', target_freq, reduction_target);
