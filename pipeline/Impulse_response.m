%% Multi-channel Room Impulse Response (sweep method, REW-free)
% Plays a log sweep out of ONE chosen output channel and records multiple mics.
% Computes IR via deconvolution and saves results.
%
% Requires: Audio Toolbox

clear; clc;

%% ---------------- User settings ----------------
fs = 48000;                 % Use the native rate of your interface if possible
frame_size = 1024;
T  = 6.0;                   % sweep duration (seconds) 5-10s is typical
f1 = 20;                    % sweep start freq
f2 = 20000;                 % sweep end freq (keep <= fs/2 * 0.9)
fadeT = 0.05;               % fade in/out (sec) to avoid clicks
preSilence = 0.5;           % silence before sweep (sec)
postSilence = 1.0;          % silence after sweep (sec)
playLevel = 0.15;           % output level (0..1). Keep conservative to avoid clipping
outChan = 1;                % which speaker channel to excite (1..Nout) e.g. center often 3
nOut = 4;                   % how many output channels your device exposes (7.1 => 8)
nIn  = 4;                   % UMC404HD => 4 inputs (set to what you use)

saveDir = fullfile(pwd, "ImpulseResponses");
if ~exist(saveDir, "dir"), mkdir(saveDir); end

%% ---------------- Build excitation (log sweep) ----------------
Nsweep = round(T*fs);
t = (0:Nsweep-1).' / fs;

% Log sweep: instantaneous phase
K = T / log(f2/f1);
L = 2*pi*f1*K;
sweep = sin(L*(exp(t./K) - 1));

% Fade to avoid clicks
Nfade = round(fadeT*fs);
w = ones(size(sweep));
w(1:Nfade) = linspace(0,1,Nfade);
w(end-Nfade+1:end) = linspace(1,0,Nfade);
sweep = sweep .* w;

% Add pre/post silence
x = [zeros(round(preSilence*fs),1); sweep; zeros(round(postSilence*fs),1)];

% Normalize and set safe playback level
x = x ./ max(abs(x)+eps);
x = playLevel * x;

% Create multi-output buffer (only outChan active)
xOut = zeros(length(x), nOut);
xOut(:, outChan) = x;

%% ---------------- Create inverse filter for deconvolution ----------------
% For a log sweep, the inverse filter is the time-reversed sweep with amplitude correction.
% This is a standard practical approach and works well for room IR measurement.
%
% Amplitude correction term:
% For exponential sweep, inverse can be approximated by reversing sweep and applying exp(t/K).
invCorr = exp((0:Nsweep-1).' / (fs*K));      % grows with time
invSweep = flipud(sweep) .* invCorr;
invSweep = invSweep ./ max(abs(invSweep)+eps);

% Put inverse in a full-length vector aligned to the sweep segment only
hInv = [invSweep; zeros(length(x) - Nsweep, 1)];

%% ---------------- Audio I/O ----------------

% Create audio device reader for input
reader = audioDeviceReader;
InDevices = getAudioDevices(reader);

% Search device list for 4 input Behringer interface
idy = find(contains(lower(string(InDevices)),"in 1-4 (behringer"), 1, "first");

% Terminates if the microphone interface isn't detected
if ~isempty(idy)
    input_name = InDevices(idy);
    disp("Using: " + input_name)
else
    disp("Behringer input interface not detected... terminating now!")
    return
end

% Configure the audio device reader
reader.Device = string(input_name);
reader.SampleRate = fs;
reader.SamplesPerFrame = frame_size;
reader.NumChannels = 4;

% Map microphones to actual inputs of hardware
ref_chan = 1;
err_chans = [2 3 4];
M = numel(err_chans);
alpha = ones(M,1)/M;

% Create audio device writer for output
writer = audioDeviceWriter;
devices = getAudioDevices(writer); 

% Search the device list for Sony Speaker Array
idx = find(contains(lower(string(devices)),"out 1-4 (behringer"), 1, "first");

% Terminates if the speaker array isn't connected
if ~isempty(idx)
    outName = devices(idx);
    disp("Using: " + outName)
else
    disp("Behringer output not detected... terminating now!")
    return
end

% Configure the audio device writer for 7 channel output with the specified sample rate
writer.Device = string(outName);
writer.SampleRate = fs;
writer.Driver = "WASAPI";

% Prime / warm-up a bit (some devices glitch on first frame)
frameSize = 1024;
setup(reader); 
setup(writer, zeros(frame_size, nOut));
for k=1:10
    y0 = reader();
    writer(zeros(frameSize,nOut));
end

%% ---------------- Playback + record loop ----------------
N = length(xOut);
Nframes = ceil(N/frameSize);

rec = zeros(Nframes*frameSize, nIn);
outIdx = 1;

fprintf("Recording %d input ch while exciting output ch %d...\n", nIn, outChan);

for k = 1:Nframes
    % Slice output frame (pad with zeros at end)
    idx = outIdx:min(outIdx+frameSize-1, N);
    frame = zeros(frameSize, nOut);
    frame(1:numel(idx), :) = xOut(idx, :);

    % Output then input (ordering can matter by driver; this is usually OK)
    writer(frame);
    recFrame = reader();
    rec(outIdx:outIdx+frameSize-1, :) = recFrame;

    outIdx = outIdx + frameSize;
end

release(reader); release(writer);

rec = rec(1:N, :);

%% ---------------- Quick clipping check ----------------
peakIn = max(abs(rec), [], 1);
fprintf("Input peak abs per channel: "); fprintf("%.3f ", peakIn); fprintf("\n");
if any(peakIn > 0.98)
    warning("One or more input channels likely clipped. Lower playLevel or input gain and re-measure.");
end

%% ---------------- Deconvolution to get IRs ----------------
% IR per mic: conv(recording, inverse) -> approximates room IR
% Use FFT convolution for speed
nFFT = 2^nextpow2(length(rec(:,1)) + length(hInv) - 1);
HINV = fft(hInv, nFFT);

ir = zeros(nFFT, nIn);
for m = 1:nIn
    Y = fft(rec(:,m), nFFT);
    ir(:,m) = real(ifft(Y .* HINV));
end

% Trim to a reasonable IR window (e.g., first 1.5s after direct sound)
irLenSec = 1.5;
irLen = round(irLenSec*fs);
irTrim = ir(1:irLen, :);

% Normalize each IR for saving (optional)
irTrimNorm = irTrim ./ (max(abs(irTrim),[],1) + eps);

%% ---------------- Save files ----------------
tag = sprintf("outCh_%02d_%s", outChan, string(datetime("now","Format","dd-MMM-uuuu")));
rawWav = fullfile(saveDir, "recording_" + tag + ".wav");
audiowrite(rawWav, rec, fs);

for m = 1:nIn
    fn = fullfile(saveDir, sprintf("IR_mic%02d_%s.wav", m, tag));
    audiowrite(fn, irTrimNorm(:,m), fs);
end

matFn = fullfile(saveDir, "IR_" + tag + ".mat");
save(matFn, "fs","T","f1","f2","preSilence","postSilence","playLevel","outChan","nOut","nIn", ...
            "x","xOut","rec","hInv","ir","irTrim","irTrimNorm");

%% ---------------- Plot (sanity check) ----------------
tIR = (0:size(irTrim,1)-1).'/fs;

figure; 
plot(tIR, irTrimNorm);
grid on;
xlabel("Time (s)"); ylabel("Amplitude (normalized)");
title(sprintf("Room IRs (Output ch %d excited)", outChan));
legend(compose("Mic %d", 1:nIn), "Location","northeast");
