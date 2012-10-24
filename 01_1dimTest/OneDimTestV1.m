clear all
close all
%% create input signal
% create sinusoidal signal
start = 0;
stop = 2*pi;
samplePerN = 400;
N = 2;
x = linspace(start, N*stop, N*samplePerN);
y1 = sin(x);
%add white gauss noise to it
snr = 0.2; %signal to noise ratio in dB
y2 = awgn(y1,snr);

plot(x,y1,x,y2);
legend('original','gauss-noise')

%%  