clear;
clc;

%% FxNLMS algorithm
% Using 1 mic and 1 speaker
%% Simulated noise signals for initial testing
sim_s = [0.5 0.5 -0.3 -0.3 -0.2 -0.2];                 
sim_f = [0.1 -0.1 0.2 -0.2 0.3 -0.3 0.15 -0.15];        
sim_p = conv(sim_s,sim_f);                                       


%% Primary path

[p_raw,Fs] = audioread('C Feb 5.wav');

% Same check as before (Should be 4800 samples)
fprintf('Loaded IR: %d samples, Fs = %d Hz, duration = %.3f s\n', ...
    length(p_raw), Fs, length(p_raw)/Fs);

% Find first peak (Should be at 1 second as REW always has a delay of 1sec)
[~, idxPeak] = max(abs(p_raw));
fprintf('Peak at sample %d (%.3f ms)\n', idxPeak, 1000*idxPeak/Fs);

% Take a window around the peak
pre  = round(0.01 * Fs);   % 5 ms before peak
post = round(0.02 * Fs);   % 85 ms after peak

i1 = max(1, idxPeak - pre);
i2 = min(length(p_raw), idxPeak + post);

h_win = p_raw(i1:i2);

% Now choose fixed tap length for Simulink
N = 1024;
if length(h_win) < N
    h_win = [h_win; zeros(N-length(h_win),1)];
else
    h_win = h_win(1:N);
end

% Assign and normalise Primary path to be used in FIR filters
P_hat = h_win;
P_hat = P_hat / (max(abs(P_hat)) + eps);  % normalise
P_hat = P_hat / (norm(P_hat) + eps);   % energy normalise

% Plot
t_ms = (0:length(P_hat)-1)/Fs*1000;
figure; plot(t_ms,P_hat); grid on;
xlabel('Time (ms)'); ylabel('Amplitude');
title('Aligned + trimmed primary path IR (P\_hat)');

P_hat = P_hat(:).';

%% Secondary path

[s_raw,Fs] = audioread('L Feb 5.wav');

% Quick check to see if files have been exported correctly (4800 samples)
fprintf('Loaded IR: %d samples, Fs = %d Hz, duration = %.3f s\n', ...
    length(s_raw), Fs, length(s_raw)/Fs);

% Find first peak (Should be at 1 second as REW always has a delay of 1sec)
[~, idxPeak] = max(abs(s_raw));
fprintf('Peak at sample %d (%.3f ms)\n', idxPeak, 1000*idxPeak/Fs);

% Take a window around the peak
pre  = round(0.01 * Fs);   % 10 ms before peak
post = round(0.02 * Fs);   % 20 ms after peak

i1 = max(1, idxPeak - pre);
i2 = min(length(s_raw), idxPeak + post);

h_win = s_raw(i1:i2);

% Now choose fixed tap length for Simulink
N = 1024; % Might need to adjust depending on computing power
if length(h_win) < N
    h_win = [h_win; zeros(N-length(h_win),1)];
else
    h_win = h_win(1:N);
end

S_hat = h_win;
S_hat = S_hat / (max(abs(S_hat)) + eps);  % peak normalise
S_hat = S_hat / (norm(S_hat) + eps);  % energy normalise

% Plot
t_ms = (0:length(S_hat)-1)/Fs*1000;
figure; plot(t_ms,S_hat); grid on;
xlabel('Time (ms)'); ylabel('Amplitude');
title('Aligned + trimmed secondary path IR (S\_hat)');

S_hat = S_hat(:).';

%% Secondary estimate path

% % Take a window around the peak
% pre  = round(0.02 * Fs);   % 20 ms before peak
% post = round(0.04 * Fs);   % 40 ms after peak
% 
% i1 = max(1, idxPeak - pre);
% i2 = min(length(s_raw), idxPeak + post);
% 
% h_win = s_raw(i1:i2);
% 
% % Now choose fixed tap length for Simulink
% N = 1024; % Might need to adjust depending on computing power
% if length(h_win) < N
%     h_win = [h_win; zeros(N-length(h_win),1)];
% else
%     h_win = h_win(1:N);
% end
% 
% E_hat = h_win;
% E_hat = E_hat / (max(abs(E_hat)) + eps);  % peak normalise
% E_hat = E_hat / (norm(E_hat) + eps);  % energy normalise
% 
% % Plot
% t_ms = (0:length(E_hat)-1)/Fs*1000;
% figure; plot(t_ms,E_hat); grid on;
% xlabel('Time (ms)'); ylabel('Amplitude');
% title('Aligned + trimmed secondary path IR (S\_hat)');
% 
% E_hat = E_hat(:).';