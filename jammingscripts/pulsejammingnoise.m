%% PULSE JAMMING NOISE - TRULY MOVING PULSES IN TIME
clear; clc;
%release(tx);% Ensure any previous transmitter object is released

%% PARAMETERS
fs = 1e6;
fc = 2.4e9;    %ISM BAND 
pulse_width = 200e-6;       %pulse width in seconds is 200 ms
% num_pulses = 5;
duty_cycle = 1.0;   %duty cycle 1.0 = 100%
tx_gain = -40;
rx_gain = 40;
num_frames = 30;        %total no of jamming + reception cycles
frame_duration = 50e-3;  % 50 ms per frame (duration of each frame)

% -----Convert pulse width and frame duration into samples------
pulse_samples = round(pulse_width * fs);
frame_samples = round(frame_duration * fs);

% Compute how many pulses can fit in a frame according to duty cycle
total_pulse_time = duty_cycle * frame_duration;
num_pulses = floor(total_pulse_time / pulse_width);


%% SETUP RECEIVER
rx = sdrrx('Pluto');
rx.CenterFrequency = fc;
rx.BasebandSampleRate = fs;

rx.SamplesPerFrame = frame_samples;
rx.OutputDataType = 'double';
rx.Gain = rx_gain;

figure;     %create a new figure window for visualization 

for i = 1:num_frames
     %% Generate NEW moving pulses each frame
    tx_signal = zeros(frame_samples, 1, 'like', complex(single(0)));
     % Preallocate a frame of zeros (no signal initially)

     % Random, sorted list of pulse starting indices within frame
    available_range = frame_samples - pulse_samples;
    pulse_starts = sort(randperm(available_range, num_pulses));

      % Insert random complex noise pulses at different positions
    for k = 1:num_pulses
        idx_start = pulse_starts(k);
        idx_end = idx_start + pulse_samples - 1;
        amplitude = 1.0;
        pulse = amplitude * (randn(pulse_samples,1,'single') + 1j*randn(pulse_samples,1,'single'));   % Complex Gaussian noise
        tx_signal(idx_start:idx_end) = pulse;
    end

    %% TRANSMIT
    tx = sdrtx('Pluto');
    tx.CenterFrequency = fc;
    tx.BasebandSampleRate = fs;
    tx.Gain = tx_gain;

    transmitRepeat(tx, tx_signal);
    pause(frame_duration * 1.1);  % Allow RX to receive full frame

    %% RECEIVE
    rxData = rx();

    %% VISUALIZE
  subplot(2,1,1);
    plot(real(rxData));
   title(sprintf('Frame %d - Pulses in Time (Truly Moving)', i));
    xlabel('Samples'); ylabel('Amplitude'); grid on;

 subplot(2,1,2);
   spectrogram(rxData, 512, 400, 512, fs, 'yaxis');
    title('Spectrogram (Live)');
    drawnow;

    %% RELEASE TRANSMITTER
    release(tx);
end

%% RELEASE RECEIVER
release(rx);
disp("✅ All frames complete with visibly moving pulse locations.");
