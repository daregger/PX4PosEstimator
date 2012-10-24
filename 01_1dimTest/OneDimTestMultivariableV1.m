%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pos / Vel / Acc traking of a car
% http://khinhooi.blogspot.ch/2012/02/kalman-filtering-for-dummies-part-ii.html
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

%% Model
T = 0.1;      %Timestep
m = 1000;   %[kg]
F = 40;    %[N]
A = [1 T 0.5*T^2; 0 1 T; 0 0 0];            % system dynamics
B = [0 0 F/m]';                             % input
H = [1 0 0; 0 1 0; 0 0 1];                  % we measure every state 

%% create measurement signal
start = 0;
stop = 500;
xt = linspace(start, stop, stop);

% constant Force over a certain time
F_start = 100;
F_stop = 300;
u(1:stop) = 0;
u(F_start:F_stop) = 1;
%plot(xt,u);
init_pos = 1;
init_vel = 0;
init_acc = 0;
x_original(1:3,1) = [init_pos init_vel init_acc]';
for k = 2:stop
    x_original(1:3,k) = A*x_original(1:3,k-1)+B*u(k-1);
end
% add white gauss noise to it
snr1 = 10; % signal to noise ratio in dB
snr2 = 10;
snr3 = 40;
z(1,:) = awgn(x_original(1,:),snr1);
z(2,:) = awgn(x_original(2,:),snr2);
z(3,:) = awgn(x_original(3,:),snr3);

%%
Q = [0.1 0 0; 0 0.1 0; 0 0 1];                              %process noise variance
%R = [std(z(1,:)) 0 0; 0 std(z(2,:)) 0; 0 0 std(z(3,:))] %measurement noise variance
R = [10 0 0; 0 10 0; 0 0 10];
%% Initial condition
x(1:3,1) = [1 1 0.05]';                                % just a initial guess
P = eye(3);                                         % not equal 0!
%%  Kalman Filter iteration
for k = 2:stop
    %Step 1 prediction/a priori
    x_s(:,k) = A*x(1:3,k-1)+B*u(k);                 %project state ahead
    P_s = A*P*A'+Q;                                 %project error covariance ahead
    %Step 2 correction/a posteriori
    K = P_s*H'*inv(H*P_s*H+R);                      %compute kalman gain
    x(:,k) = x_s(:,k)+K*(z(:,k)-H*x_s(:,k));          %update estimate via z
    P = (eye(3)-K*H)*P_s;                           %update error covariance
end

%% Integral and Derivate of noisy measurement
v = diff(z(1,:));
p = cumsum(z(2,:));

%% Plot
figure(1);
subplot(4,1,1);
plot(xt,x_original(1,:),xt,z(1,:),xt,x(1,:),xt,p);
ylabel('Pos');
legend('original','gauss-noise measurement','estimate','vel integral','Location','NorthEastOutside')
subplot(4,1,2);
plot(xt,x_original(2,:),xt,z(2,:),xt,x(2,:),xt(1:499),v);
ylabel('Vel');
legend('original','gauss-noise measurement','estimate','pos derivative','Location','NorthEastOutside')
subplot(4,1,3);
plot(xt,x_original(3,:),xt,z(3,:),xt,x(3,:));
ylabel('Acc');
subplot(4,1,4);
plot(xt,u);
ylabel('Force input');