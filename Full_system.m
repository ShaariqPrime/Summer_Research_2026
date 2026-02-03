%%
% _____      _   _ _     __  __ ____       _        _   _           
% |  ___|_  _| \ | | |   |  \/  / ___|     / \   ___| |_(_)_   _____ 
% | |_  \ \/ /  \| | |   | |\/| \___ \    / _ \ / __| __| \ \ / / _ \
% |  _|  >  <| |\  | |___| |  | |___) |  / ___ \ (__| |_| |\ V /  __/
% |_|  _/_/\_\_| \_|_____|_|  |_|____/  /_/   \_\___|\__|_| \_/ \___|
% | \ | | ___ (_)___  ___                                            
% |  \| |/ _ \| / __|/ _ \                                           
% | |\  | (_) | \__ \  __/                                           
% |_|_\_|\___/|_|___/\___|   _ _       _   _                         
%  / ___|__ _ _ __   ___ ___| | | __ _| |_(_) ___  _ __              
% | |   / _` | '_ \ / __/ _ \ | |/ _` | __| |/ _ \| '_ \             
% | |__| (_| | | | | (_|  __/ | | (_| | |_| | (_) | | | |            
%  \____\__,_|_| |_|\___\___|_|_|\__,_|\__|_|\___/|_| |_|            

clear; 
clc;
close all;

%% ---------------- User settings ----------------
% sample rate (Hz)
fs = 48000;
% Number of samples per frame
frame_size = 256;
% length of adaptive control filter W(z)
Lw = 1024;                  
% step size
mu = 5e-4;                
% NLMS regularization
delta = 1e-3;               
% Leakage factor (0 = none). Helps prevent drift in real systems.
leak = 1e-5;

%% Estimate secondary path from impulse response

[s_raw,Fs] = audioread('Impulse responses/Spath_SL_28_1.wav');

% Quick check to see if files have been exported correctly (4800 samples)
fprintf('Loaded IR: %d samples, Fs = %d Hz, duration = %.3f s\n', ...
    length(s_raw), Fs, length(s_raw)/Fs);

% Find first peak (Should be at 1 second as REW always has a delay of 1sec)
[~, idxPeak] = max(abs(s_raw));
fprintf('Peak at sample %d (%.3f ms)\n', idxPeak, 1000*idxPeak/Fs);

% Take a window around the peak
pre  = round(0.005 * Fs);   % 5 ms before peak
post = round(0.050 * Fs);   % 50 ms after peak

i1 = max(1, idxPeak - pre);
i2 = min(length(s_raw), idxPeak + post);

h_win = s_raw(i1:i2);

% Now choose fixed tap length 
N = 1024; % Might need to adjust depending on computing power
if length(h_win) < N
    h_win = [h_win; zeros(N-length(h_win),1)];
else
    h_win = h_win(1:N);
end

S_hat = h_win;
% peak normalise
S_hat = S_hat / (max(abs(S_hat)) + eps);
% energy normalise (not necessary as negligible effect)
% S_hat = S_hat / (norm(S_hat) + eps);  

% Plot to verify secondary estimate
% t_ms = (0:length(S_hat)-1)/Fs*1000;
% figure; plot(t_ms,S_hat); grid on;
% xlabel('Time (ms)'); ylabel('Amplitude');
% title('Aligned + trimmed secondary path IR (S\_hat)');

%% I/O setup

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
    disp("Behringer interface not detected... terminating now")
    return
end

% Configure the audio device reader
reader.Device = string(input_name);
reader.SampleRate = fs;
reader.SamplesPerFrame = frame_size;
reader.NumChannels = 4;

% Map microphones to actual inputs of hardware
err_chan = 1;
ref_chan = 2;

% Create audio device writer for output
writer = audioDeviceWriter;         % create object
devices = getAudioDevices(writer);  % cell array of device names

% Search the device list for Sony Speaker Array
idx = find(contains(lower(string(devices)),"sony avamp"), 1, "first");

% Terminates if the speaker array isn't connected
if ~isempty(idx)
    hdminame = devices(idx);
    disp("Using: " + hdminame)
else
    disp("Sony speaker array not detected... terminating now")
    return
end

% Configure the audio device writer with the specified sample rate
writer.Device = string(hdminame);
writer.SampleRate = fs;
writer.ChannelMappingSource = "Property";
writer.ChannelMapping = 1:8;

% Channel mapping = 1 FL, 2 FR, 3 C, 4 SUB, 5 BL, 6 BR, 7 SL, 8 SR
noise_chan = 3;
cancel_chan = 7;

%% ---------------- State / buffers ----------------
% adaptive filter weights W(z)
w = zeros(Lw,1);                        
% buffer for x into W(z)
x_buf      = zeros(Lw-1,1);             
% buffer for x into S_hat(z)
xhat_buf   = zeros(Lw,1);               
% buffer for y into S(z) (plant)
y_buf_S    = zeros(length(S_hat)-1, 1);

% ---- Extra secondary-path delay (measured) ----
Dsec = 2542;                  
xF_delay_buf = zeros(Dsec,1); % delay line for filtered-x (x_f)

% Randomly seed 
rng(0);
disp("FxNLMS in progress...")

%% Scope setup

% scope = timescope( ...
%     'SampleRate', fs, ...
%     'TimeSpan', 10, ...                 
%     'BufferLength', fs*10, ...
%     'NumInputPorts', 2, ...
%     'ShowLegend', true, ...
%     'ChannelNames', {'d(n) baseline', 'e(n) ANC'}, ...
%     'YLimits', [-0.1 0.1]);

%% Data recording
Trec = 60;
Nrec = round(Trec * fs);

micLog   = zeros(Nrec, reader.NumChannels, 'single');  % raw interface inputs
xLog     = zeros(Nrec, 1, 'single');                   % noise 
errLog   = zeros(Nrec, 1, 'single');                   % error
yLog     = zeros(Nrec, 1, 'single');                   % cancel sent to cancel speaker

writeIdx = 1;
y_frame = zeros(frame_size, 1);

%% ---------------- Main ANC loop ----------------

phase = round(30 * fs/ frame_size);

for i = 1:2
    fprintf('\n========== Phase %d ==========\n', i);
    if i == 1
        disp('Collecting baseline (no ANC)...');
    else
        disp('ANC active - adapting weights...');
    end
    
    for k = 1:phase
        % Initialise error and reference mic
        in = reader();
        em = in(:,err_chan);
        x = in(:,ref_chan);

        % Data recording
        idx2 = writeIdx + frame_size - 1;
        if idx2 <= Nrec
            micLog(writeIdx:idx2, :) = single(in);
            xLog(writeIdx:idx2)      = single(x);
            yLog(writeIdx:idx2)      = single(y_frame);
            writeIdx = idx2 + 1;
        else
            break
        end
        
        % Phase 1: Baseline (no control)
        if i == 1
            % Arbitrary array used for data recording
            y_frame = zeros(frame_size, 1);
            % Monitor correlation every 50 frames
            if mod(k, 50) == 0
                for ch = 1:4
                    rms_ch = sqrt(mean(in(:,ch).^2));
                    var_ch = var(in(:,ch));
                    fprintf('Channel %d: RMS=%.6f, Var=%.6f\n', ch, rms_ch, var_ch);
                end
                fprintf('---\n')
            end
        % Phase 2: ANC active
        else
            % Generate control signal y using current weights
            [y_frame, x_buf] = filter(w, 1, x, x_buf);
            % Saturation for the output noise
            ymax = 0.2;
            y_frame = max(min(y_frame, ymax), -ymax);
            % Filter reference signal through secondary path estimate (short FIR tail)
            [x_f_tail, y_buf_S] = filter(S_hat, 1, x, y_buf_S);
            
            % Apply measured pure delay Dsec to the filtered-x signal
            x_f = xF_delay_buf(1:frame_size);
            xF_delay_buf = [xF_delay_buf(frame_size+1:end); x_f_tail];
            
            % Normalisation FxNLMS
            for n = 1:frame_size
                xhat_buf = [x_f(n); xhat_buf(1:end-1)];
                denom = (xhat_buf.'*xhat_buf) + delta;
                w = (1 - leak)*w - (mu/denom) * em(n) * xhat_buf;
            end
        end
        
        % Create empty array to match channel size
        y_out = zeros(frame_size, 8);
        % Concatenate arrays for output
        y_out(:,noise_chan) = zeros(size(x));    % Not needed
        y_out(:,cancel_chan) = y_frame;
        % Write audio data to specified dev
        writer(y_out);
        
        % Scope update for each phase
        % if i == 1
        %     scope(em, zeros(size(em)));
        % else
        %     scope(zeros(size(em)), em);
        % end       
    end

    if writeIdx > Nrec
        break;
    end
end

%% Save results

audiowrite("mics_raw.wav", micLog, fs);
audiowrite("ref_noise.wav", xLog, fs);
audiowrite("err_noise.wav", errLog, fs);
audiowrite("y_cancel.wav", yLog, fs);
disp("Sucessfully saved audio files");
