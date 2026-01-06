clear;
clc;

%% Parameters
centerFreq = 2.4e9;        % Center frequency (2.4 GHz for example)
sampleRate = 6e6;          % 6 MHz sample rate
toneFreq = 1e6;            % Tone at 1 MHz offset from center
txGain = -10;              % Pluto transmit gain
rxGain = 30;               % Pluto receive gain
numSamples = 2^14;         % Samples per frame

%% Generate Single Tone Signal
t = (0:numSamples-1)' / sampleRate;
tone = exp(1i*2*pi*toneFreq*t);  % Complex exponential (sine wave at 1 MHz)
txSignal = 0.8 * tone;           % Scale to avoid clipping

%% Create Pluto Transmitter
tx = sdrtx('Pluto', ...
    'CenterFrequency', centerFreq, ...
    'BasebandSampleRate', sampleRate, ...
    'Gain', txGain);

%% Transmit Continuously
transmitRepeat(tx, txSignal);

%% Create Pluto Receiver
rx = sdrrx('Pluto', ...
    'CenterFrequency', centerFreq, ...
    'BasebandSampleRate', sampleRate, ...
    'GainSource', 'Manual', ...
    'Gain', rxGain, ...
    'SamplesPerFrame', numSamples, ...
    'OutputDataType', 'double');

pause(1); % Let system stabilize

%% Receive Signal
rxSignal = rx();
rxSignal = rxSignal - mean(rxSignal); % Remove DC offset

%% TIME DOMAIN PLOT
figure;
subplot(2,1,1);
plot(real(rxSignal(1:1000)));  % Plot real part
title('Received Tone - Time Domain');
xlabel('Sample Index');
ylabel('Amplitude');
grid on;

%% FREQUENCY DOMAIN PLOT
nfft = 2^nextpow2(numSamples);
f = linspace(-sampleRate/2, sampleRate/2, nfft)/1e6;  % MHz scale
spectrum = fftshift(20*log10(abs(fft(rxSignal, nfft))));

subplot(2,1,2);
plot(f, spectrum);
title('Received Tone - Frequency Domain');
xlabel('Frequency (MHz)');
ylabel('Magnitude (dB)');
ylim([-20 70]);
grid on;

%% Cleanup
release(tx);
release(rx);
