clear; clc;

jammerMode = 'bandlimited';   % 'wideband' or 'bandlimited'

% === STEP 1: Set frequency to your router's channel ===
centerFreq = 2.412e9;       % Example: Wi-Fi Channel 6
sampleRate = 20e6;          % Full Wi-Fi channel
txGain     = 0;             % Max Gain (0 dB = full gain)
numSamples = 2^18;
amplitude  = 1;           % Strong amplitude

% === STEP 2: SDR Setup ===
tx = sdrtx('Pluto', ...
    'CenterFrequency', centerFreq, ...
    'BasebandSampleRate', sampleRate, ...
    'Gain', txGain, ...
    'ChannelMapping', 1);

% === STEP 3: Generate complex Gaussian noise ===
txSignal = amplitude * (randn(numSamples,1) + 1j*randn(numSamples,1));

% === STEP 4: Apply optional bandpass filter ===
if strcmpi(jammerMode, 'bandlimited')
    disp(' Band-limited noise: 6–14 MHz portion of Wi-Fi band...');
    bpFilt = designfilt('bandpassfir', ...
        'FilterOrder', 400, ...
        'CutoffFrequency1', 4e6, ...
        'CutoffFrequency2', 6e6, ...
        'SampleRate', sampleRate);
    txSignal = filter(bpFilt, txSignal);
end

% === STEP 5: Visualize Spectrum (Optional) ===
% spectrumAnalyzer = dsp.SpectrumAnalyzer('SampleRate', sampleRate, ...
%     'Title', 'Transmitted Spectrum');
% spectrumAnalyzer(txSignal); pause(3);  % Optional plot

% === STEP 6: Transmit repeatedly ===
disp('Transmitting noise... Press Ctrl+C to stop.');
transmitRepeat(tx, txSignal);
