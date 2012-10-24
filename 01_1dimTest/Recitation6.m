% Recursive Filtering and Estimation, Spring 2012
% Exercise class, 25.04.12
%
% Script to run example "Kalman Filter: Observability".
%
% Markus Hehn, hehnm(at)ethz.ch

%% System parameters
A = 0.5;
eta = 0.1;
Q = 1;
R = 1;

%% Initial state estimate variance
P0 = 0;

%% Execution parameters
n = 50; % Number of steps to simulate

%% Run the filter
P_m = zeros(n,1);
P_p = zeros(n,1);

P_m(1) = P0; %Initialize with P0

for(i=2:n)
  % S1: Prediction Step
  P_p(i) = A^2*P_m(i-1) + Q;
  
  % S2: Measurement Update
  P_m(i) = 1/(1/P_p(i) + eta^2/R);
end

%% Plot results
figure(5), plot(1:n, P_p);
xlabel('Time index k');
ylabel('Post-prediction variance P_p(k)');