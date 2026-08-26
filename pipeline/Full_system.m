%%
%  _____      _   _ _     __  __ ____       _        _   _           
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
frame_size = 480;
% length of adaptive control filter W(z)
Lw = 256;                  
% step size
mu = 5e-4;                
% NLMS regularization
delta = 1e-6;
% Leakage factor. Helps prevent drift in real systems.
leak = 1e-6;

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
    disp("Behringer interface not detected... terminating now!")
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
% writer.Driver = "WASAPI";

cancel_chans = [1 2]; 
K = numel(cancel_chans);

%% Estimate secondary path from impulse response

S_hat = cell(M,K);

for m = 1:M
    for k = 1:K
        fn = sprintf("ImpulseResponses/IR_mic%02d_outCh_%02d_19-Feb-2026.wav", ...
            err_chans(m), cancel_chans(k));
        [s_raw,Fs] = audioread(fn);
    
        % Quick check to see if files have been exported correctly (4800 samples)
        fprintf('Loaded IR: %d samples, Fs = %d Hz, duration = %.3f s\n', ...
            length(s_raw), Fs, length(s_raw)/Fs);
    
        % Find first peak
        [~, idxPeak] = max(abs(s_raw));
        fprintf('Peak at sample %d (%.3f ms)\n', idxPeak, 1000*idxPeak/Fs);
        
        % Take a window around the peak
        pre  = round(0.002 * Fs);   % 2 ms before peak
        post = round(0.030 * Fs);   % 300 ms after peak
    
        i1 = max(1, idxPeak - pre);
        i2 = min(numel(s_raw), idxPeak + post);
    
        h_win = s_raw(i1:i2);
        N = 512;
        if numel(h_win) < N
            h_win = [h_win; zeros(N-numel(h_win),1)];
        else
            h_win = h_win(1:N);
        end

        % peak normalise
        S_hat{m,k} = h_win / (max(abs(h_win)) + 1e-12);
        % energy normalise
        % s = S_hat{m,k}(:);
        % s = s / (sqrt(mean(s.^2)) + 1e-12); 
        % S_hat{m,k} = s;

        % Plot to verify secondary estimate
        t_ms = (0:length(S_hat{m,k})-1)/Fs*1000;
        figure; plot(t_ms,S_hat{m,k}); grid on;
        xlabel('Time (ms)'); ylabel('Amplitude');
        title('Aligned + trimmed secondary path IR (S\_hat)');
    end
end



%% ---------------- State / buffers ----------------
% Preallocate arrays so MATLAB doesn't slow down during the ANC process

% adaptive filter weights W(z)
w = zeros(Lw, K);  
% buffer for x into W(z)
x_buf = zeros(Lw-1, K);
% buffer for x into S_hat(z)
% xhat_buf = zeros(Lw, M, K);

% buffer for y into S(z) (plant)
y_buf_S = cell(M,K);

for m = 1:M
    for k = 1:K
        y_buf_S{m,k} = zeros(length(S_hat{m,k})-1, 1);
    end
end

% Randomly seed 
rng(0);
disp("FxNLMS in progress...")

%% Data recording

% Change number for seconds to record
% Must equal total 'phase' from below otherwise will terminate ANC early
Nrec = round(80 * fs);

micLog   = zeros(Nrec, reader.NumChannels, 'single');  % raw interface inputs
xLog     = zeros(Nrec, 1, 'single');                   % noise 
errLog   = zeros(Nrec, M, 'single');                   % error
yLog     = zeros(Nrec, K, 'single');                   % cancel sent to cancel speaker

writeIdx = 1;
count1 = 0;
count2 = 0;

%% ---------------- Main ANC loop ----------------

% Time for Baseline and ANC phase (number indicates seconds)
phase = round(40 * fs/ frame_size);

[bLP,aLP] = butter(4,700/(fs/2));
yLP_state = zeros(max(length(aLP),length(bLP))-1,K);

% Dynamic mu state (optional but helpful)
mu_dyn = mu;
mu_min = 1e-8;
mu_up  = 1.002;
mu_dn  = 0.2;
e_smooth = 0;
alpha_e = 0.05;
ring_ratio = 1.12;

% block-adaptation state (no xhat_buf needed)
Px = 1e-3 * ones(M,K);   % running power estimate of x_f per (m,k)
alphaP = 0.9;            % smoothing for power

% Arbitrary array used for data recording
y_frame = zeros(frame_size, K);

for i = 1:2
    fprintf('\n========== Phase %d ==========\n', i);
    if i == 1
        disp('Collecting baseline (no ANC)...');
    else
        disp('ANC active - adapting weights...');
    end
    
    for p = 1:phase
        % Initialise error and reference mic
        in = reader();
        E = in(:,err_chans);
        rm = in(:,ref_chan);

        % Phase 1: Baseline (no control)
        if i == 1
            % During baseline phase, output is 0
            y_frame = zeros(frame_size, K);
            % Monitor correlation roughly every 1 second (only for debugging mics)
            if mod(p, 100) == 0
                for ch = 1:4
                    rms_ch = sqrt(mean(in(:,ch).^2));
                    var_ch = var(in(:,ch));
                    fprintf('Channel %d: RMS=%.6f, Var=%.6f\n', ch, rms_ch, var_ch);
                end
                count1 = count1 + 1;
                fprintf('Seconds %d:, ---\n', count1)
            end
        % Phase 2: ANC active
        else
            % Saturation for the output noise
            ymax = 0.02;
            % Generate control output for given reference signal
            for k = 1:K
                % Generate control signal y using current weights
                [y_frame(:,k), x_buf(:,k)] = filter(w(:,k), 1, rm, x_buf(:,k));
                [y_frame(:,k), yLP_state(:,k)] = filter(bLP,aLP,y_frame(:,k),yLP_state(:,k));
                y_frame(:,k) = ymax * tanh(y_frame(:,k)/ymax);   % soft saturation
            end

            % Predict how the reference propogates through the room using
            % S_hat (Speaker-Mic path)
            x_f = zeros(frame_size, M, K);
            for k = 1:K
                for m = 1:M
                    % Filter reference signal through secondary path estimate (short FIR tail)
                    [x_f_tail, y_buf_S{m,k}] = filter(S_hat{m,k}, 1, rm, y_buf_S{m,k});
                    x_f(:,m,k) = x_f_tail(1:frame_size);
                end
            end
            % ---------------- Update using PREVIOUS frame ---------------- 
            e_frame = rms(E(:));
            if e_smooth == 0
                e_smooth = e_frame;
            else
                e_smooth = (1-alpha_e)*e_smooth + alpha_e*e_frame;
            end
            if e_frame > ring_ratio * e_smooth
                mu_dyn = max(mu_dyn * mu_dn, mu_min);
            else
                mu_dyn = min(mu_dyn * mu_up, mu);
            end

            for k = 1:K 
                grad = zeros(Lw,1); 
                for m = 1:M
                    xf = x_f(:,m,k);
                    em = E(:,m);

                    Px(m,k) = alphaP*Px(m,k) + (1-alphaP)*(mean(xf.^2) + delta);


                    % Build regression matrix X (frame_size x Lw), causal tapped delays
                    X = zeros(frame_size, Lw);
                    for n = 1:frame_size
                        i0 = max(1, n-Lw+1);
                        len = n - i0 + 1;
                        X(n,1:len) = flipud(xf(i0:n));
                    end

                    grad = grad + alpha(m) * (X.' * em) / Px(m,k);
                end
            w(:,k) = (1 - leak)*w(:,k) - mu_dyn * grad;
    
            % safety cap (prevents runaway ringing)
            wmax = 2.0;
            wn = norm(w(:,k));
            if wn > wmax
                w(:,k) = (wmax/wn) * w(:,k);
            end
            end
            if mod(p,100) == 0 
                ctrl_rms = sqrt(mean(y_frame(:).^2)); 
                err_rms = sqrt(mean(E(:).^2)); 
                w_norm = norm(w(:)); 
                fprintf("ctrl_rms=%.4f err_rms=%.4f ||w||=%.3f\n",ctrl_rms,err_rms,w_norm); 
                fprintf("rms(x_f)=%.6f\n",rms(x_f(:))); 
                count2 = count2 + 1; 
                fprintf('Seconds %d:, ---\n', count2); 
            end
        end 

        % Data recording 
        idx2 = writeIdx + frame_size - 1; 
        if idx2 <= Nrec 
            micLog(writeIdx:idx2, :) = single(in); 
            xLog(writeIdx:idx2) = single(rm); 
            errLog(writeIdx:idx2, :) = single(E); 
            yLog(writeIdx:idx2, :) = single(y_frame); 
            writeIdx = idx2 + 1; 
        else 
            break 
        end 
        
        % Create empty array to match channel size 
        y_out = zeros(frame_size, 2); 
        % Concatenate arrays for output 
        for k = 1:K 
            y_out(:,cancel_chans(k)) = y_frame(:,k); 
        end 
        % y_out(:,noise_chan) = zeros(size(rm)); % For feedforward = 0 
        % Write audio data to specified device 
        writer(y_out); 
    end
    % if writeIdx > Nrec
    %     break;
    % end
end

%% Save results

disp("ANC loop finished, saving audio files...");
audiowrite("mics_raw.wav", micLog, fs);
audiowrite("ref_noise.wav", xLog, fs);
audiowrite("err_noise.wav", errLog, fs);
audiowrite("y_cancel.wav", yLog, fs);
disp("Sucessfully saved audio files");
