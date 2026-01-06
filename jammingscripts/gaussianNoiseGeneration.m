%% PARAMETERS
fs = 1e6; % Sampling rate: 1 MHz
fc = 915e6; % Center frequency: 915 MHz (ISM band - safer)
gainTx = -20; % Transmit gain (low to reduce interference)
gainRx = 30; % Receive gain (high to amplify weak signal)

%% GENERATE GAUSSIAN NOISE
N = fs * 2; % Generate 2 seconds worth of samples
% randn() generates Gaussian-distributed real and imaginary parts
txSignal = complex(randn(N,1), randn(N,1)); 
% Normalize the signal to keep values between -1 and +1
txSignal = txSignal / max(abs(txSignal)); 

%% SETUP TRANSMITTER
% Configure Pluto SDR to transmit the Gaussian noise
tx = sdrtx('Pluto', ...
'CenterFrequency', fc, ...
'BasebandSampleRate', fs, ...
'Gain', gainTx); % Low power to avoid RF pollution

% Start transmitting signal on repeat
transmitRepeat(tx, txSignal); 
disp('Transmitting Gaussian noise...');

%% SETUP RECEIVER
% Configure Pluto SDR to receive the transmitted noise
rx = sdrrx('Pluto', ...
'CenterFrequency', fc, ...
'BasebandSampleRate', fs, ...
'SamplesPerFrame', 4096, ...
'Gain', gainRx, ...
'OutputDataType', 'double');

%% CONTINUOUSLY RECEIVE AND PLOT
figure; % Open a new figure window for plotting

for i = 1:100 % Run the loop 200 times (~20 seconds of data)
rcv = rx(); % Receive 4096 samples

% TIME DOMAIN PLOT
subplot(2,1,1); % Top half of the figure
plot(real(rcv(1:1000))); % Plot first 1000 samples (I component)
title('Time Domain - Received Noise');
xlabel('Samples'); ylabel('Amplitude');

% FREQUENCY DOMAIN PLOT
subplot(2,1,2); % Bottom half of the figure
NFFT = 1024; % FFT size
spectrum = abs(fftshift(fft(rcv, NFFT))); % Take FFT and center zero freq
spectrum = 20*log10(spectrum / max(spectrum)); % Normalize and convert to dB
f = linspace(-fs/2, fs/2, NFFT); % Frequency axis in Hz

plot(f/1e6, spectrum); % Plot in MHz
title('Frequency Domain - Received Noise');
xlabel('MHz'); ylabel('Magnitude (dB)');
ylim([-60 5]); % Set y-axis range for better visibility

drawnow; % Force immediate update of the plots
end

%% CLEANUP
release(rx); % Free the receiver
release(tx); % Free the transmitter
disp('Done.');