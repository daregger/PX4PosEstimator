%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thrust angel pos estimator V2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%% Model
f = 70;
f_pos = 1;
dt = 1/f;   %Timestep
m = 0.448;   %[kg]
angle_x_deg = 15;
angle_x_rad = angle_x_deg*pi/180;
angle_y_deg = 15;
angle_y_rad = angle_y_deg*pi/180;
thrust_x = 400;   %is between 0-511
thrust_y = 400;   %is between 0-511
F_thrust_x = thrust2force(thrust_x);
F_thrust_y = thrust2force(thrust_y);
F_x = F_thrust_x*sin(angle_x_rad);              %[N]
F_y = F_thrust_y*sin(angle_y_rad); 
acc_x_max = F_x/m;
acc_y_max = F_y/m;

A_x = [1 dt; 0 1];                         % system dynamics continuous
B_x = [0.5*dt*dt dt]'*acc_x_max;           % input
H_x = [1 0; 0 0];                          
velocity_observe_x = 0;

A_y = [1 dt; 0 1];                         % system dynamics
B_y = [0.5*dt*dt dt]'*acc_y_max;           % input
H_y = [1 0; 0 0];                          
velocity_observe_y = 0;

%% create measurement signal
start = 0;
stop = 40*f;

sigma_x = 1;
sigma_y = 1;

xt = linspace(start, stop, stop);
pos_mask_x = zeros(1,stop);
pos_mask_y = zeros(1,stop);
for i = 1:floor(f/f_pos):stop
    pos_mask_x(1,i) = 1;
    pos_mask_y(1,i) = 1;
end
%%%%%%%%%% x %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% constant Force over a certain time

F_x_start = 10*f;
F_x_stop = 11*f;
u_x(1:stop) = 0;
F_x_diff = F_x_stop - F_x_start;
for i = 1:F_x_diff
    u_x(i+F_x_start) = sin(i*pi/F_x_diff); 
end
for i = 1:F_x_diff
    u_x(i+2*F_x_start) = sin(i*pi/F_x_diff+pi); 
end
%plot(xt,u_x);
init_pos_x = 0;
init_vel_x = 0;
x_original_x(:,1) = [init_pos_x init_vel_x]';

for k = 2:stop
    x_original_x(:,k) = A_x*x_original_x(:,k-1)+B_x*u_x(k-1);
end
%plot(xt,x_original_x)
% add white gauss noise to it
n1x = sigma_x*randn(1,stop);
z_x(1,:) = x_original_x(1,:) + n1x(1,:);
%plot(xt,z_x(1,:));
%var(n1x)

%%%%%%%%%% y %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% constant Force over a certain time
F_y_start = 14*f;
F_y_stop = 15*f;
u_y(1:stop) = 0;
F_y_diff = F_y_stop - F_y_start;
for i = 1:F_y_diff
    u_y(i+F_y_start) = sin(i*pi/F_y_diff); 
end
for i = 1:F_y_diff
    u_y(i+2*F_y_start) = sin(i*pi/F_y_diff+pi); 
end
%plot(xt,u_y);
init_pos_y = 0;
init_vel_y = 0;
x_original_y(:,1) = [init_pos_y init_vel_y]';

for k = 2:stop
    x_original_y(:,k) = A_y*x_original_y(:,k-1)+B_y*u_y(k-1);
end
% add white gauss noise to it
n1y = sigma_y(1).*randn(1,stop);
z_y(1,:) = x_original_y(1,:) + n1y(1,:);
%plot(xt,z_y(1,:));

%% generate acceleration vector over time
a_x_original = zeros(1,stop);
a_y_original = zeros(1,stop);
for i = 1:stop
    a_x_original(1,i) = acc_x_max*u_x(i);
    a_y_original(1,i) = acc_y_max*u_y(i);
end
sigma_x_a = 0.5;
sigma_y_a = 0.5;
a_x_offset = 0;
a_y_offset = 0;
a_x(1,:) = a_x_original + sigma_x_a.*randn(1,stop) + a_x_offset;
a_y(1,:) = a_y_original + sigma_y_a.*randn(1,stop) + a_y_offset;
%plot(xt,a_x); ylabel('m/s^2');

%% Initial condition
x_x(:,1) = [0 0]';                                % just a initial guess
P_x = 10*ones(2);                                  % not equal 0!

x_y(:,1) = [0 0]';                                % just a initial guess
P_y = 10*ones(2);                                  % not equal 0!

%%
q_x = [1e-5 1e-5]';
q_y = [1e-8 1e-5]';

% %% Kalman Filter iteratiion x           old buggy one
% for k = 1:stop-1
%     %[x_x(:,k+1),P_x] = positionKalmanFilter1D(x_x(:,k),P_x,velocity_observe_x,pos_mask_x(k),F_x/m*u_x(k+1),z_x(:,k+1),sigma_x,q_x,dt);
%     [x_x(:,k+1),P_x] = positionKalmanFilter1D(x_x(:,k),P_x,velocity_observe_x,pos_mask_x(k),0,z_x(:,k+1),sigma_x,q_x,dt);
%     %[x_y(:,k+1),P_y] = positionKalmanFilter1D(x_y(:,k),P_y,velocity_observe_y,pos_mask_y(k),F_y/m*u_y(k+1),z_y(:,k+1),sigma_y,q_y,dt);
%     [x_y(:,k+1),P_y] = positionKalmanFilter1D(x_y(:,k),P_y,velocity_observe_y,pos_mask_y(k),0,z_y(:,k+1),sigma_y,q_y,dt);
% end

%% Kalman Filter iteration
for k = 1:stop-1
    %[x_x(:,k+1),P_x] = positionKalmanFilter1D(x_x(:,k),P_x,velocity_observe_x,pos_mask_x(k),a_x(1,k),[z_x(1,k) 0]',[sigma_x 0]',q_x,dt);
    [x_x(:,k+1),P_x] = positionKalmanFilter1D(x_x(:,k),P_x,velocity_observe_x,pos_mask_x(k),a_x(1,k),[z_x(1,k) 0]',[sigma_x 0]',q_x,dt);
    %[x_x(:,k+1),P_x] = positionKalmanFilter1DONLYPOS(x_x(:,k),P_x,velocity_observe_x,pos_mask_x(k),0,z_x(1,k),sigma_x,q_x,dt);
    %[x_y(:,k+1),P_y] = positionKalmanFilter1D(x_y(:,k),P_y,velocity_observe_y,pos_mask_y(k),a_y(1,k),[z_y(1,k) 0]',[sigma_y 0]',q_y,dt);
    [x_y(:,k+1),P_y] = positionKalmanFilter1D(x_y(:,k),P_y,velocity_observe_y,pos_mask_y(k),a_y(1,k),[z_y(1,k) 0]',[sigma_y 0]',q_y,dt);
end
%plot(xt,pos_mask_x)

%% Integral and Derivate of noisy measurement
%v_x = diff(z_x(1,:));
%p_x = cumsum(z_x(2,:));
%v_y = diff(z_y(1,:));
%p_y = cumsum(z_y(2,:));

%% Plot help
pos_plot_x = pos_mask_x.*z_x(1,:);
pos_plot_x(pos_plot_x==0) = NaN;

pos_plot_y = pos_mask_y.*z_y(1,:);
pos_plot_y(pos_plot_y==0) = NaN;

%% 3D plot
figure('units','normalized','outerposition',[0 0 1 1])
figure(1);
plot3(xt,x_original_y(1,:),x_original_x(1,:))
hold on
plot3(xt,z_y(1,:),z_x(1,:),'g');
plot3(xt,x_y(1,:),x_x(1,:),'r');
plot3(xt,pos_plot_y,pos_plot_x,'mo')
hold off
xlabel('Time')
ylabel('Y Pos');
zlabel('X Pos')
grid on
legend('original','gauss-noise measurement','estimate');

%% single axis plot
% x
figure('units','normalized','outerposition',[0 0 1 1])
figure(2);
subplot(3,2,1);
plot(xt,x_original_x(1,:),xt,z_x(1,:),xt,x_x(1,:),xt,pos_plot_x,'mo');
ylabel('X Pos');
legend('original','gauss-noise measurement','estimate');
subplot(3,2,3);
plot(xt,x_original_x(2,:),xt,x_x(2,:),'r');
ylabel('X Vel');
legend('original','estimate');
subplot(3,2,5);
plot(xt,a_x);
ylabel('Acc x input [m/s^2]');

% y
%figure('units','normalized','outerposition',[0 0 1 1])
figure(2);
subplot(3,2,2);
plot(xt,x_original_y(1,:),xt,z_y(1,:),xt,x_y(1,:),xt,pos_plot_y,'mo');
ylabel('Y Pos');
legend('original','gauss-noise measurement','estimate');
subplot(3,2,4);
plot(xt,x_original_y(2,:),xt,x_y(2,:),'r');
ylabel('Y Vel');
legend('original','estimate');
subplot(3,2,6);
plot(xt,a_y);
ylabel('Acc y input [m/s^2]');
