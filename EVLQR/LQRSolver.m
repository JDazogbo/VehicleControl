%% Part 6
% Filter specifications
order = 6;
cutoff = 0.70;  % Cutoff frequency in radians/sample

% Design the Butterworth filter
[b, a] = butter(order, cutoff / pi, 'low');  % Normalize cutoff frequency by pi

% Compute the frequency response
[h, w] = freqz(b, a, 8000);

% Plot the magnitude response
figure;
plot(w, abs(h), 'LineWidth', 1.5);
hold on;
xline(cutoff, '--r', 'Cutoff Frequency', 'LabelVerticalAlignment', 'middle');
title('Magnitude Response of 4th-Order Butterworth Filter');
xlabel('Frequency [radians/sample]');
ylabel('Magnitude');
grid on;
legend('Magnitude Response', 'Location', 'best');
xlim([0, pi]);
ylim([0, 1.1]);
xticks(0:pi/4:pi);
xticklabels({'0', '\pi/4', '\pi/2', '3\pi/4', '\pi'});
hold off;

%% Part D and E
% Define the parameters
M = 60;  % Length of the envelope sequence
n = 0:M; % Time indices for the envelope

% Define the envelope sequence w[n]
w = 0.54 - 0.46 * cos(2 * pi * n / M);

% Define the three narrowband pulses
x1 = w .* cos(0.2 * pi * n);               % x1[n] = w[n] * cos(0.2πn)
x2 = w .* cos(0.4 * pi * n - pi/2);         % x2[n] = w[n] * cos(0.4πn - π/2)
x3 = w .* cos(0.8 * pi * n + pi/5);         % x3[n] = w[n] * cos(0.8πn + π/5)

% Construct the complete input signal x[n]
x = [x3, zeros(1, M+1), x1, zeros(1, M+1), x2];

% Plot the input signal x[n]
figure;
subplot(2, 1, 1);
stem(0:length(x)-1, x, 'filled', 'MarkerSize', 3);
title('Waveform of Signal x[n]');
xlabel('Sample Number (n)');
ylabel('Amplitude');
grid on;

% Compute the DTFT of x[n]
N = 1024;  % Number of frequency points
X = fft(x, N);  % Compute the DFT
f = (-N/2:N/2-1) * (2 * pi / N);  % Frequency axis in radians

% Plot the magnitude of the DTFT of x[n]
subplot(2, 1, 2);
plot(f, fftshift(abs(X)));
title('Magnitude of DTFT of x[n]');
xlabel('\omega (radians)');
ylabel('|X(e^{j\omega})|');
grid on;

% Design the Butterworth filter (from previous steps)
omega_p = 0.2 * pi;  % Passband edge frequency
omega_s = 0.3 * pi;  % Stopband edge frequency
Rp = 1;  % Passband ripple in dB
As = 15; % Stopband attenuation in dB

% Determine the order and cutoff frequency
[N, Wn] = buttord(omega_p / pi, omega_s / pi, Rp, As);

% Design the Butterworth filter
[b, a] = butter(N, Wn);

% Filter the input signal x[n] using the designed filter
y = filter(b, a, x);

% Plot the filtered signal y[n]
figure;
subplot(2, 1, 1);
stem(0:length(y)-1, y, 'filled', 'MarkerSize', 3);
title('Filtered Signal y[n]');
xlabel('Sample Number (n)');
ylabel('Amplitude');
grid on;

% Compute the DFT of the filtered signal y[n]
Y = fft(y, N);  % Compute the DFT

% Plot the magnitude of the DFT of y[n]
subplot(2, 1, 2);
plot(f, fftshift(abs(Y)));
title('Magnitude of DFT of Filtered Signal y[n]');
xlabel('\omega (radians)');
ylabel('|Y(e^{j\omega})|');
grid on;
