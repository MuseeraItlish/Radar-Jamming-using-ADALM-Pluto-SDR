%% Doppler Radar using ADALM-PLUTO
% Parameters
fc = 2.4e9;              % Carrier frequency (Hz)
fs = 1e6;                % Baseband sample rate (Hz)
c = 3e8;                 % Speed of light (m/s)
v = 10;                  % Target velocity (m/s)
samplesPerFrame = 10000;
t = (0:samplesPerFrame-1)/fs;

% Doppler frequency
fd = 2*v*fc/c;

% Pulse parameters
pulsePeriod = 10e-3;             % Pulse repetition interval (10 ms)
pulseWidth = 200e-6;             % Pulse width (200 us)
pulsePeriodSamples = round(pulsePeriod * fs);
pulseSamples = round(pulseWidth * fs);

% Generate pulse envelope
pulseEnvelope = zeros(size(t));
for startIdx = 1:pulsePeriodSamples:length(t)
    endIdx = min(startIdx + pulseSamples - 1, length(t));
    pulseEnvelope(startIdx:endIdx) = 1;
end

% Generate pulsed radar signal
f_pulse = 100e3;   % Baseband pulse tone frequency
pulsedSignal = pulseEnvelope .* exp(1j*2*pi*f_pulse*t);  % Complex pulsed tone
pulsedSignal = complex( pulsedSignal.');  % Column vector

%Transmitter Configuration
tx = sdrtx('Pluto','RadioID', 'usb:0', ...
    'CenterFrequency', fc,'BasebandSampleRate', fs,'Gain', -10);

%Receiver Configuration
rx = sdrrx('Pluto','RadioID', 'usb:0','CenterFrequency', fc, 'BasebandSampleRate', fs, 'SamplesPerFrame', samplesPerFrame,'GainSource', 'Manual', 'Gain', 40);

% Transmit + Receive
transmitRepeat(tx, pulsedSignal);

disp("Running Doppler radar...");

runtime = tic;
i = 0;
while toc(runtime) < 10  % Run for 10 seconds
    i = i + 1;

    % Receive echo
    rxData = double(rx());
    % === J/S Ratio Calculation ===
    % Estimate signal power from samples inside expected echo window (pulses)
    echoPower = mean(abs(rxData).^2);  % total power in received frame

    % Estimate jamming power:
    % If jammer is wideband noise, it's spread across whole frame
    % So total power = echo + jammer
    % If you have a separate measurement of jamming-only (e.g., run without TX on),
    % you can subtract it from total power to isolate signal

    % For simplicity, let's assume you run two cases:
    % 1. With jammer off  → estimate echoPower
    % 2. With jammer on   → totalPower = signal + jammer

  % Convert to dB
echoPower_dB = 10 * log10(echoPower);
% Collect reference signal power (clean)

rxData = double(rx());
noJammerPower = mean(abs(rxData).^2);
fprintf("Reference power recorded: %.2f dB\n", 10*log10(noJammerPower));


if exist('noJammerPower', 'var')
    % Compute jammer power
    jammerPower = echoPower - noJammerPower;
    if jammerPower < 0
        jammerPower = 1e-12; % Avoid negative values
    end
    jammerPower_dB = 10 * log10(jammerPower);
    noJammerPower_dB = 10 * log10(noJammerPower);

    % Compute J/S ratio in dB
    J_to_S_dB = jammerPower_dB - noJammerPower_dB;

    fprintf("Received Power: %.2f dB | J/S Ratio: %.2f dB\n", echoPower_dB, J_to_S_dB);
else
    noJammerPower = echoPower;
    fprintf("Reference Power (no jammer): %.2f dB\n", echoPower_dB);
end


    % Plot time domain
    figure(1);
    plot((t+i)*1e3, real(rxData), 'b'); hold on;
    plot((t+i)*1e3, imag(rxData), 'r'); hold off;
    xlabel('Time (ms)');
    ylabel('Amplitude');
    title('Received Signal - Time Domain');
    legend('Real', 'Imag');
    grid on;
    drawnow;
    pause(0.01);
end

% Cleanup
release(tx);
release(rx);

% Frequency Domain Analysis (Doppler Shift)
N = length(rxData);
f_axis = (-fs/2:fs/N:fs/2 - fs/N)/1e3;  % in kHz
spectrum = fftshift(fft(rxData)/N);
figure(2);
plot(f_axis, abs(spectrum), 'b');
xlabel('Frequency (kHz)');
ylabel('Magnitude');
title('Received Signal - Frequency Domain (Doppler)');
grid on;

% %% MULTIPLE CHIRPS
% %% Doppler / FMCW radar with multiple chirps 
% 
% % -parameters
% fc  = 2.4e9;           % RF centre
% fs  = 1e6;             % BB sample rate
% c   = 3e8;
% M   = 4;               % *** number of chirps per PRI ***
% pulsePeriod = 10e-3;   % 10 ms PRI
% pulseWidth   = 200e-6; % 200 µs per chirp
% f_start      = -200e3; % -200 kHz (relative to BB centre)
% f_end        =  200e3; %  200 kHz
% 
% 
% samplesPerChirp   = round(pulseWidth  * fs);
% samplesPerFrame   = round(pulsePeriod * fs);
% t_frame           = (0:samplesPerFrame-1).'/fs;     % column
% 
% % Build multi‑chirp envelope 
% pulseEnvelope = zeros(samplesPerFrame,1);
% for m = 0:M-1
%     s = m * samplesPerChirp + 1;   % chirp start sample
%     e = min(s + samplesPerChirp - 1, samplesPerFrame);
%     pulseEnvelope(s:e) = 1;
% end
% 
% % Generate one chirp's complex exponential 
% t_chirp = (0:samplesPerChirp-1).'/fs;
% k       = (f_end - f_start)/pulseWidth;             % sweep rate (Hz/s)
% oneChirp= exp(1j*2*pi*(f_start*t_chirp + 0.5*k*t_chirp.^2));
% 
% %Tile the chirp M times inside the frame 
% pulsedSignal = zeros(samplesPerFrame,1);
% for m = 0:M-1
%     idx = (1:samplesPerChirp) + m*samplesPerChirp;
%     pulsedSignal(idx) = oneChirp;
% end
% pulsedSignal = pulsedSignal .* pulseEnvelope;       % (redundant but explicit)
% 
% % transmit / recieve
% tx = sdrtx('Pluto','CenterFrequency',fc,'BasebandSampleRate',fs,'Gain',-10);
% rx = sdrrx('Pluto','CenterFrequency',fc,'BasebandSampleRate',fs,...
%            'SamplesPerFrame',samplesPerFrame,'GainSource','Manual','Gain',40);
% 
% transmitRepeat(tx,pulsedSignal);
% 
% disp("Running multi‑chirp radar …");
% runtime = tic;
% while toc(runtime) < 10
%     rxData = rx();
% 
%     % Quick range/Doppler visual (slow‑time FFT then fast‑time FFT)
%     data_mat = reshape(rxData(1:M*samplesPerChirp),samplesPerChirp,[]); % [fast × slow]
%     rngFFT   = fft(data_mat,[],1);               % range dimension
%     dopFFT   = fftshift(fft(rngFFT,[],2),2);     % Doppler dimension
% 
%     imagesc(abs(dopFFT)); colormap turbo; colorbar;
%     xlabel('Chirp index (slow‑time)'); ylabel('Range bin (fast‑time)');
%     title('Range‑Doppler map');
%     drawnow;
% end
% release(tx); release(rx);
%%
% parameters
fc              = 2.4e9;      % RF centre frequency (Hz)
fs              = 1e6;        % Baseband sample‑rate (Hz)
pulseWidth      = 100e-6;     % Width of one pulse  (100 µs)
PRI             = 1e-3;       % Pulse‑repetition‑interval (1 ms)
numPulsesFrame  = 50;         % Pulses per transmit buffer
txGain_dB       = -10;        % Pluto output gain (dB)

%  parameters ---------------------------------------
samplesPerPulse = round(pulseWidth * fs);
samplesPerPRI   = round(PRI        * fs);
samplesPerFrame = numPulsesFrame * samplesPerPRI;

t_frame         = (0:samplesPerFrame-1).' / fs;   % column vector

% 3. Build baseband pulse train -------------------------------
pulseTrain = zeros(samplesPerFrame,1);

for k = 0:numPulsesFrame-1
    s = k * samplesPerPRI + 1;                     % start index
    e = s + samplesPerPulse - 1;                   % end index
    pulseTrain(s:e) = 1;                           % 1 = "on"
end

% Optional: apply a carrier or PN code inside each pulse
% baseTone = exp(1j*2*pi*200e3*t_frame);           % 200 kHz BB tone
% pulseTrain = pulseTrain .* baseTone;

% 4. Configure PlutoSDR transmitter ---------------------------
tx = sdrtx('Pluto', ...
           'CenterFrequency'   , fc, ...
           'BasebandSampleRate', fs, ...
           'Gain'              , txGain_dB);

%5. Start transmitting ---------------------------------------
pulseTrain = complex(pulseTrain);
transmitRepeat(tx, pulseTrain);
disp("► Multi‑pulse stream is now repeating from PlutoSDR.");

%%6. Stop with   
release(tx);   
% when finished