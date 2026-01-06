% sweepnoise_tx.m - Transmit sweep noise using Pluto SDR

fs = 2e6; % Sample rate
T = 10e-3; % Duration of 1 sweep
N = round(fs * T); % Samples per sweep
fStart = 0;
fEnd = fs / 2;

% Time vector
t = (0:N-1)' / fs;

% Generate chirp-based sweep noise
sweepReal = chirp(t, fStart, T, fEnd, 'linear');

% Add randomness (to make it noisy)
sweepNoise = sweepReal + 0.1 * randn(size(sweepReal)); % Optional noise overlay

% Apply window to reduce spectral leakage
w = hann(N);
sweepNoise = sweepNoise .* w;

% Normalize
sweepNoise = sweepNoise / max(abs(sweepNoise));

% Repeat to make continuous
sweepNoise = repmat(sweepNoise, 100, 1);

% Convert to complex float
sweepNoise = complex(single(sweepNoise));

% Transmit setup
tx = sdrtx('Pluto', ...
'CenterFrequency', 2.4e9, ...
'BasebandSampleRate', fs, ...
'Gain', 0);

% Transmit continuously
transmitRepeat(tx, sweepNoise); % Will keep transmitting until released

disp('Transmitting sweep noise...');

% sweepnoise_rx.m - Receive and visualize sweep noise using Pluto SDR

fs = 2e6;

% Receiver setup
rx = sdrrx('Pluto', ...
'CenterFrequency', 2.4e9, ...
'BasebandSampleRate', fs, ...
'SamplesPerFrame', 8192, ...
'OutputDataType', 'double');

% Wait a bit for stability
pause(0.1);

% Capture one frame
rxSig = rx();
release(rx);

% Time-domain plot
figure;
plot(real(rxSig));
title('Time-Domain Received Sweep Noise');
xlabel('Sample Index'); ylabel('Amplitude');

% Frequency domain (Spectrogram)
figure;
spectrogram(rxSig, 512, 400, 1024, fs, 'centered');
title('Spectrogram of Received Sweep Noise');
