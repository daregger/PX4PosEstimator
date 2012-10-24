clear all
clc

%timeline and sampling period, Ts
Ts = 0.01;
t = 0:Ts:10;

%true parameters position, velocity and acceleration
x = cos(t);
dx = -sin(t);
ddx = -cos(t);

%add noise to the "measurements"
for i=1:length(t)
    x_noise(i) = x(i) + (randn());
    dx_noise(i) = dx(i) + (randn());
    ddx_noise(i) = ddx(i) + (randn());
end

%Kalman filter initialization
%initialize the states, here, the initial value is incorrect
q = [0; 0; 1];  
%initialize the estimate covariance with nonzero values.
P = eye(3);       

%F and H are the model parameters where
F = [1 Ts 0.5*Ts*Ts; 0 1 Ts; 0 0 1];
H = [1 0 0; 0 1 0; 0 0 1];

%Q and R both the process and measurement noise covariances
Q = [ 0 0 0; 0 0 0; 0 0 1];
R = [1 0 0; 0 1 0; 0 0 1];

%Kalman iterate
for i = 1:length(t)-1
    %predict stage
   %Step 1 prediction/a priori
    q(:,i+1) = F*q(1:3,i);                 %project state ahead
    P_s = F*P*F'+Q;                     %project error covariance ahead
    %Step 2 correction/a posteriori
    
    z=[x_noise(i);dx_noise(i);ddx_noise(i)];
    
    K = P_s*H'*inv(H*P_s*H+R);                      %compute kalman gain
    q(:,i+1) = q(:,i)+K*(z(i)-H*q(:,i));          %update estimate via z
    P = (eye(3)-K*H)*P_s;                           %update error covariance
end

%Plotting the data
subplot(311)
my_plot = plot(t,x,t,q(1,:),t,x_noise,':');
set(my_plot, 'linewidth', 1);
legend('Position','Estimate Position','Position with Noise');
subplot(312)
my_plot = plot(t,dx,t,q(2,:),t, dx_noise,':');
set(my_plot, 'linewidth', 1);
legend('Velocity','Estimate Velocity','Velocity with Noise');
subplot(313)
my_plot = plot(t,ddx,t,q(3,:),t,ddx_noise,':');
set(my_plot, 'linewidth', 1);
legend('Acceleration','Estimate Acceleration','Acceleration with noise');