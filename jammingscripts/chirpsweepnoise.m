% sweepnoise_tx.m - Transmit sweep noise using Pluto SDR
%a chirp signal that is slightly randomied to behave like  sweep noise

%% --------------------- Parameters ---------------------
fs = 2e6; % Sample rate
T = 10e-3; % Duration of 1 sweep
N = round(fs * T); % Samples per sweep
fStart = 0;
fEnd = fs / 2;


%% --------------------- Time Vector ---------------------
% column vecotr of time samples from 0 to T
t = (0:N-1)' / fs;  %time base for 1 sweep


%% --------------------- Generate Chirp ---------------------
% Generate linear chirp-based sweep noise from 0 to 1MHz (since fs=2e6 and
% fend = fs/2) over a duration of 10 ms
sweepReal = chirp(t, fStart, T, fEnd, 'linear');

%% --------------------- Add Gaussian Noise ---------------------
% Add Gaussian noise to the chirp signal to make it sweep noise
sweepNoise = sweepReal + 0.1 * randn(size(sweepReal));  %0.1 is the noise amplitude 

%% --------------------- Apply Windowing ---------------------
% Apply window to reduce spectral leakage and smooth the edges 
w = hann(N);        %generate han window
sweepNoise = sweepNoise .* w;   %apply the window to the signal 

%% --------------------- Normalize ---------------------
% normalize the signal amplitude to avoid clipping during transmission
sweepNoise = sweepNoise / max(abs(sweepNoise));


%% --------------------- Repeat the Sweep ---------------------
% repeat the sweep 100 times to make the signal long enough for continuous playback
sweepNoise = repmat(sweepNoise, 100, 1);      %concatenate 100 identical sweeps

%% --------------------- Convert to Complex Format ---------------------
%convert the signal to complex I/Q form 
sweepNoise = complex(single(sweepNoise));

%% --------------------- Configure Pluto SDR Transmitter ---------------------
tx = sdrtx('Pluto', ...
'CenterFrequency', 2.4e9, ...
'BasebandSampleRate', fs, ...
'Gain', 0);     %tx gain = 0 is highest transmission gain 

%% --------------------- Transmit Continuously ---------------------
% Continuously transmit the sweep noise until released manually
transmitRepeat(tx, sweepNoise); % Will keep transmitting until released

disp('Transmitting sweep noise...');

%% --------------------- Parameters ---------------------

% sweepnoise_rx.m - Receive and visualize sweep noise using Pluto SDR

% sweepnoise_rx_live.m - Real-time sweep noise visualization
fs = 2e6;
frameLen = 8192;

%% --------------------- Configure Pluto SDR Receiver ---------------------
% Receiver setup
rx = sdrrx('Pluto', ...
    'CenterFrequency', 2.4e9, ...
    'BasebandSampleRate', fs, ...
    'SamplesPerFrame', frameLen, ...
    'OutputDataType', 'double');


%% --------------------- Create Visualization Layout ---------------------
% Create a figure with scrolling spectrogram
figure;
subplot(2,1,1);
hTime = plot(nan, nan);
title('Time-Domain Signal');
xlabel('Sample Index'); ylabel('Amplitude');

subplot(2,1,2);
hSpec = spectrogram(randn(frameLen,1), 512, 400, 1024, fs, 'centered');
title('Spectrogram (Frequency Sweep Visualization)');

disp('Receiving and displaying sweep noise live... Press Ctrl+C to stop.');


%% --------------------- Real-Time Processing Loop ---------------------
% Real-time loop
while true
    rxSig = rx();

    % Update time-domain plot
    subplot(2,1,1);
    set(hTime, 'YData', real(rxSig), 'XData', 1:length(rxSig));
    drawnow limitrate;

    % Update spectrogram
    subplot(2,1,2);
    spectrogram(rxSig, 512, 400, 1024, fs, 'centered');
    title('Spectrogram of Received Sweep Noise');
    drawnow limitrate;
end
