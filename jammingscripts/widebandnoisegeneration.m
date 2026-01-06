% Create PlutoSDR transmitter and receiver objects
tx = sdrtx('Pluto', 'RadioID', 'usb:0');
rx = sdrrx('Pluto', 'RadioID', 'usb:0');

% Configure transmitter and receiver parameters
centerFrequency = 2.4e9; % Center frequency (e.g., 2.4 GHz, adjust as needed)
sampleRate = 1e6; % Sample rate (1 MHz, adjust for wideband, max ~61.44e6)
gain = 0; % TX gain in dB (adjust between -89 to 0, check legal limits)
rxGain = 20; % RX gain in dB (adjust between 0 to 70)
tx.CenterFrequency = centerFrequency;
tx.BasebandSampleRate = sampleRate; % Corrected property
tx.Gain = gain;
rx.CenterFrequency = centerFrequency;
rx.BasebandSampleRate = sampleRate; % Corrected property
rx.GainSource = 'manual';
rx.Gain = rxGain;

% Generate wideband noise
numSamples = 10000; % Number of samples to transmit/receive
noise = complex(randn(numSamples, 1), randn(numSamples, 1)); % Complex Gaussian noise
noise = noise / max(abs(noise)); % Normalize to prevent clipping
noise = 0.5 * noise;

% Transmit the noise continuously
disp('Transmitting and receiving wideband noise...');
transmitRepeat(tx, noise); % Start continuous transmission

% Receive and visualize the noise
receivedData = capture(rx, numSamples, 'Samples'); % Capture received samples

% Convert received data to complex double and compute magnitude
receivedData = complex(double(real(receivedData)), double(imag(receivedData))); % Ensure double precision
receivedMagnitude = abs(receivedData); % Compute magnitude of complex data

% Plot the received signal (magnitude over time)
figure;
plot(receivedMagnitude);
title('Received Wideband Noise Magnitude');
xlabel('Sample Index');
ylabel('Magnitude');
grid on;

% Stop transmission (optional, run manually if needed)
% release(tx);
% release(rx);