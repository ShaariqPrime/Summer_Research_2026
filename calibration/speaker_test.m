%% Initialise audio output (need to write and release)
writer = audioDeviceWriter;         % create object
devices = getAudioDevices(writer);  % cell array of device names
% disp(devices);

idx = find(contains(lower(string(devices)),"sony avamp"), 1, "first");
hdminame = devices(idx);
disp("Using: " + hdminame)

fs = 48000;         % Normal sampling rate for HDMI devices
frame_size = 1024;
t = 0:1/fs:1-1/fs;
audioData = 0.3*sin(2*pi*440*t)';

% Configure the audio device writer with the specified sample rate and frame size
writer.Device = string(hdminame);
writer.SampleRate = fs;
writer.ChannelMappingSource = "Property";
writer.ChannelMapping = 1:8;

% Test to find which speaker maps to which channel
% Results: 1 FL, 2 FR, 3 C, 4 SUB, 5 BL, 6 BR, 7 SL, 8 SR
for c = 1:8
    fprintf("Output channel %d...\n",c);

    for k = 1:15                    % Duration of sound played
        Y = zeros(fs, 8);           % Create empty array to match channel size
        Y(:,c) = audioData;         % Concatenate sound signal to be played
        writer(Y);                  % Write audio data to specified device
    end
    pause(0.4)
    release(writer);                % Release after every output
end

% Release the audio device writer after all channels have been tested
release(writer);
disp("Audio device writer released.");


%% Initialise audio input

% Create audio device reader for input
reader = audioDeviceReader;
InDevices = getAudioDevices(reader);
% disp(InDevices)

idy = find(contains(lower(string(InDevices)),"in 1-4"), 1, "first");
input_name = InDevices(idy);
disp("Using: " + input_name)

reader.Device = string(input_name);
reader.SampleRate = fs;
reader.SamplesPerFrame = frame_size;
reader.NumChannels = 4;

T = 10;         % recording time
ref_chan = 2;   % Reference mic channel

N = T * fs;     % How many samples needed
xrec = zeros(N, 1, 'single');       % Preallocate memory for the recording
count = 1;      % Writer pointer

while count <= N        % Repeat until xrec is filled
    x = reader();
    n = min(frame_size,N-count+1);      % Accounts for any overflow
    xrec(count:count+n-1) = x(1:n, ref_chan);   %Store into xrec
    count = count + n;
end

release(reader);

audiowrite("test_record_ref.wav", xrec, fs)
disp("audio written sucessfully")





