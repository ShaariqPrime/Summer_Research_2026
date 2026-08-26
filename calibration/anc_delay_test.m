%% ANC Delay Test (Robust) - cancel speaker -> error mic
% Save as: anc_delay_test.m
%
% What this does:
% 1) Opens your chosen input/output audio devices (audioDeviceReader/Writer)
% 2) Outputs a MULTI-FRAME noise burst on your cancel speaker channel
% 3) Records the error mic for a specified duration
% 4) Estimates delay using normalized cross-correlation against the exact burst sequence
%
% Tips if xcorr peak is small:
% - Confirm cancel_chan mapping is correct (speaker actually plays on that channel)
% - Confirm err_chan is correct (clap near mic; see big response on that channel)
% - Disable AVR DSP (Pure Direct / Direct; turn off room correction, EQ, surround)
% - Increase burst amplitude slightly or mic gain (avoid clipping)
% - Move error mic closer to cancel speaker for THIS TEST ONLY (boost SNR)
%
% NOTE:
% - A tone burst often gives smeared correlation in rooms. Noise burst is better.
% - HDMI + AVR can add large latency (50–200+ ms). That’s okay; we just measure it.
%
% Requires: Audio Toolbox (audioDeviceReader/audioDeviceWriter)

clear; clc;

%% ---------------- User settings ----------------
fs         = 48000;        % Hz
frame_size = 256;          % samples per frame
in_ch      = 4;            % interface input channels

err_chan   = 1;            % error mic channel index on input device (1..in_ch)
%ref_chan = 2;             % optional, not needed for this test

out_ch     = 8;            % 7.1 output
noise_chan = 3;            % unused here, but left for consistency
cancel_chan= 7;            % cancel speaker channel (1..8)

T_settle   = 0.5;          % seconds of silence to stabilize streams
T_record   = 2.5;          % seconds to record AFTER the burst is sent

burstFrames = 24;          % number of frames for the burst (24*256/48k ≈ 128 ms)
A_burst     = 0.18;        % burst amplitude (0.05..0.30). Increase carefully.
rngSeed     = 1;           % reproducible burst

maxLag_s    = 0.5;         % seconds for xcorr search window (+/-)

%% ---------------- Device selection ----------------
% Input
reader = audioDeviceReader;
InDevices = getAudioDevices(reader);

disp("Available INPUT devices:");
disp(InDevices);

% TODO: Edit this match string to your device name
inputMatch = "in 1-4";  % e.g., "umc404hd", "in 1-4", etc.

idy = find(contains(lower(string(InDevices)), lower(inputMatch)), 1, "first");
if isempty(idy)
    error("Could not find input device containing: '%s'", inputMatch);
end
input_name = string(InDevices(idy));
disp("Using INPUT: " + input_name);

reader.Device = input_name;
reader.SampleRate = fs;
reader.SamplesPerFrame = frame_size;
reader.NumChannels = in_ch;

% Output
writer = audioDeviceWriter;
OutDevices = getAudioDevices(writer);

disp("Available OUTPUT devices:");
disp(OutDevices);

% TODO: Edit this match string to your HDMI/AVR output name
outputMatch = "sony avamp";  % e.g., "sony", "hdmi", etc.

idx = find(contains(lower(string(OutDevices)), lower(outputMatch)), 1, "first");
if isempty(idx)
    error("Could not find output device containing: '%s'", outputMatch);
end
output_name = string(OutDevices(idx));
disp("Using OUTPUT: " + output_name);

writer.Device = output_name;
writer.SampleRate = fs;

% Ensure channel mapping is enabled
writer.ChannelMappingSource = "Property";
writer.ChannelMapping = 1:out_ch;

%% ---------------- Build burst sequence ----------------
rng(rngSeed);
burstSeq = A_burst * randn(frame_size*burstFrames, 1);
burstSeq = burstSeq - mean(burstSeq);

%% ---------------- Preallocate logs ----------------
N_record = round(T_record * fs);
em_log   = zeros(N_record, 1, 'single');
yref_log = zeros(N_record, 1, 'single');   % reference sequence for correlation (burst at start)

% Put the known burst sequence at the start of yref_log
Lref = min(N_record, numel(burstSeq));
yref_log(1:Lref) = single(burstSeq(1:Lref));

%% ---------------- Helpers ----------------
mkOut = @(y_cancel) makeOutFrame(frame_size, out_ch, noise_chan, cancel_chan, y_cancel);

%% ---------------- Run test ----------------
disp("=== Starting delay test ===");
disp("Settling IO...");

nSettleFrames = ceil(T_settle * fs / frame_size);
for k = 1:nSettleFrames
    writer(mkOut(zeros(frame_size,1)));
    reader(); % keep pipeline flowing
end

disp("Sending multi-frame noise burst on cancel channel...");
ptr = 1;
for b = 1:burstFrames
    y_frame = burstSeq(ptr:ptr+frame_size-1);
    writer(mkOut(y_frame));
    reader(); % keep pipeline flowing
    ptr = ptr + frame_size;
end

disp("Recording error mic while outputting silence...");
writeIdx = 1;
nRecFrames = ceil(N_record / frame_size);

for k = 1:nRecFrames
    % silence output during record window
    writer(mkOut(zeros(frame_size,1)));

    in = reader();
    em = in(:, err_chan);

    idx2 = min(writeIdx + frame_size - 1, N_record);
    L = idx2 - writeIdx + 1;
    em_log(writeIdx:idx2) = single(em(1:L));

    writeIdx = idx2 + 1;
    if writeIdx > N_record
        break;
    end
end

disp("=== Recording complete ===");

%% ---------------- Delay estimation ----------------
emv = double(em_log(:));
yv  = double(yref_log(:));

% remove DC
emv = emv - mean(emv);
yv  = yv  - mean(yv);

maxLag = round(maxLag_s * fs);
[r,lags] = xcorr(emv, yv, maxLag, 'coeff');

[pk, idxPk] = max(abs(r));
D = lags(idxPk);

fprintf("\nxcorr peak = %.5f\n", pk);
fprintf("Estimated delay D = %d samples = %.2f ms\n", D, 1000*D/fs);

% Peak level check (helps confirm SNR / clipping)
pkEm = max(abs(emv));
fprintf("Error mic peak amplitude = %.6f (%.1f dBFS)\n", pkEm, 20*log10(pkEm+1e-12));

%% ---------------- Plots ----------------
t = (0:numel(emv)-1)/fs;

figure('Name','Error mic after burst','NumberTitle','off');
plot(t, emv); grid on;
xlabel('Time (s)'); ylabel('Error mic (em)');
title(sprintf('Error mic response | xcorr peak=%.5f | D=%d samples (%.2f ms)', pk, D, 1000*D/fs));

figure('Name','xcorr(em, yref)','NumberTitle','off');
plot(lags/fs*1000, r); grid on;
xlabel('Lag (ms)'); ylabel('xcorr coeff');
title('Normalized cross-correlation: em vs yref');

%% ---------------- Save logs for offline inspection ----------------
outFile = "delay_test_logs.mat";
save(outFile, "fs", "frame_size", "burstFrames", "A_burst", "err_chan", "cancel_chan", ...
    "em_log", "yref_log", "pk", "D");
disp("Saved logs to: " + outFile);

disp("Done.");

%% ---------------- Local function ----------------
function y_out = makeOutFrame(frame_size, out_ch, noise_chan, cancel_chan, y_cancel)
    y_out = zeros(frame_size, out_ch);
    if ~isempty(y_cancel)
        y_out(:, cancel_chan) = y_cancel;
    end
    % noise_chan left as zeros (unused). Keep for consistency.
end
