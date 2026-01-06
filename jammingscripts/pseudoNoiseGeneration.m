%% CLEANUP
clear tx rx
try, release(tx); end
try, release(rx); end

%% PARAMETERS
fs = 1e6;               % Sample rate
fc = 915e6;             % Center frequency
gainTx = -40;           % Transmit gain
gainRx = 10;            % Receive gain
frameSize = 65536;      % Signal length (must be long enough)

%% GENERATE PSEUDO-NOISE SIGNAL
% Step 1: Binary sequence
pnBits = randi([0 1], frameSize, 1);         % 0s and 1s

% Step 2: Map to BPSK (0 → -1, 1 → +1)
pnMapped = 2 * double(pnBits) - 1;           % Now type is double

% Step 3: Convert to complex explicitly
txSignal = complex(pnMapped, zeros(size(pnMapped)));  % Force complex format

% Step 4: Normalize (avoid clipping)
txSignal = txSignal / max(abs(txSignal));

% Step 5: Cast to correct datatype (complex double)
txSignal = complex(double(txSignal));  % ✅ Ensures type is complex double

%% TRANSMITTER SETUP
tx = sdrtx('Pluto', ...
    'CenterFrequency', fc, ...
    'BasebandSampleRate', fs, ...
    'Gain', gainTx);

% ✅ TRANSMIT PN SIGNAL REPEATEDLY
transmitRepeat(tx, txSignal);
disp('Transmitting PN pseudo-noise...');

%% RECEIVER SETUP
rx = sdrrx('Pluto', ...
    'CenterFrequency', fc, ...
    'BasebandSampleRate', fs, ...
    'SamplesPerFrame', 4096, ...
    'Gain', gainRx, ...
    'OutputDataType', 'double');

%% LIVE PLOTTING
figure;
for i = 1:100
    rcv = rx();

    % Time domain
    subplot(2,1,1);
    plot(real(rcv));
    title('Time Domain - Received PN');
    xlabel('Samples'); ylabel('Amplitude'); ylim([-1.5 1.5]);

    % Frequency domain
    subplot(2,1,2);
    NFFT = 1024;
    spectrum = abs(fftshift(fft(rcv, NFFT)));
    spectrum = 20*log10(spectrum / max(spectrum));
    f = linspace(-fs/2, fs/2, NFFT)/1e6;
    plot(f, spectrum);
    title('Frequency Domain - Received PN');
    xlabel('Frequency (MHz)'); ylabel('Magnitude (dB)'); ylim([-60 5]);
    grid on;

    drawnow;
end

%% CLEANUP
release(tx);
release(rx);
disp('Done. SDR Released.');
