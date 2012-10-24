%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simplest constant value tracking
% http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

%% create measurement signal
start = 0;
stop = 1;
samplePerN = 500;
N = 2;
xt = linspace(start, N*stop, N*samplePerN);
% constant value
ampl = 0.5;
x_original(1:N*samplePerN) = ampl;
% add white gauss noise to it
snr = 15; % signal to noise ratio in dB
z = awgn(x_original,snr);
u(1:N*samplePerN) = 0;

%% Model
A = 1;  % cause of constant value signal
B = 0;  % cause of no input
H = 1;  % since z_k = Hx_k + v_k and measurement is state value + noise

Q = 0;
R = std(z) % standard deviation
%R = 10;

%% Initial condition
x(1) = 1; % just a initial guess
P(1) = 1;   % not equal 0!

%%  Kalman Filter iteration
for k = 2:N*samplePerN
    %Step 1 prediction/a priori
    x_s(k) = A*x(k-1)+B*u(k);                          %project state ahead
    P_s(k) = A*P(k-1)*A'+Q;                     %project error covariance ahead
    %Step 2 correction/a posteriori
    K(k) = P_s(k)*H'*inv(H*P_s(k)*H+R);         %compute kalman gain
    x(k) = x_s(k)+K(k)*(z(k)-H*x_s(k));         %update estimate via z
    P(k) = (eye(1)-K(k)*H)*P_s(k);              %update error covariance
end

plot(xt,x_original,xt,z,xt,x);
legend('original','gauss-noise measurement','estimate')
