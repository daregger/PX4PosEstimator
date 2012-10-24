%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thrust angel pos estimator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

%% Model
f = 250;
f_pos = 4;
dt = 1/f;   %Timestep
m = 1.5;   %[kg]
angle_deg = 30;
angle_rad = angle_deg*pi/180;
thrust = 20;   %is between 0-511
F_thrust = schubkurveTobi(thrust);
F_x = F_thrust*sin(angle_rad);              %[N]
A = [1 dt; 0 1];                         % system dynamics
B = [0 F_x/m]';                           % input
H = [1 0; 0 0];                         % we measure every state 
velocity_observe = 0;

%% create measurement signal
start = 0;
stop = 10*f;
xt = linspace(start, stop, stop);
pos_mask = zeros(1,stop);
for i = 1:floor(f/f_pos):stop
    pos_mask(1,i) = 1;
end
% constant Force over a certain time
F_start = 3*f;
F_stop = 4*f;
u(1:stop) = 0;
F_diff = F_stop - F_start;
for i = 1:F_diff
    u(i+F_start) = sin(i*pi/F_diff); 
end
for i = 1:F_diff
    u(i+2*F_start) = sin(i*pi/F_diff+pi); 
end
%plot(xt,u);
init_pos = 1;
init_vel = 0;
x_original(:,1) = [init_pos init_vel]';
for k = 2:stop
    x_original(:,k) = A*x_original(:,k-1)+B*u(k-1);
end
% add white gauss noise to it
sigma = [5 1.5];
n1 = sigma(1).*randn(1,stop);
n2 = sigma(2).*randn(1,stop);
z(1,:) = x_original(1,:) + n1(1,:);
z(2,:) = x_original(2,:) + n2(1,:);

%% Initial condition
x(:,1) = [0 2]';                                % just a initial guess
P = 10*ones(2);                                         % not equal 0!

%%
q = [0.3162 1];
Q = [q(1)^2*dt 0; 0 q(2)^2*dt];                                      %process noise variance
R = [sigma(1)^2 0; 0 sigma(2)^2];                       %measurement noise variance

%%  Kalman Filter iteration
for k = 1:stop-1
    %Step 1 prediction/a priori
    x_apriori = A*x(:,k)+B*u(k+1);                          %project state ahead
    P_apriori = A*P*A'+Q;                                 %project error covariance ahead
    %Step 2 correction/a posteriori
    if pos_mask(1,k+1) == 1
        K = P_apriori*H'*inv(H*P_apriori*H+R);                      %compute kalman gain
        x(:,k+1) = x_apriori+K*(z(:,k+1)-H*x_apriori);          %update estimate via z
        P = (eye(2)-K*H)*P_apriori;                              %update error covariance
    else
       x(:,k+1)=x_apriori;
       P=P_apriori;
    end        
end

%% Integral and Derivate of noisy measurement
v = diff(z(1,:));
p = cumsum(z(2,:));

%% Plot help
pos_plot = pos_mask.*z(1,:);
pos_plot(pos_plot==0) = NaN;

%% Plot
%big window
figure('units','normalized','outerposition',[0 0 1 1])
figure(1);
subplot(3,1,1);
%plot(xt,x_original(1,:),xt,z(1,:),xt,x(1,:),xt,p);
plot(xt,x_original(1,:),xt,z(1,:),xt,x(1,:),xt,pos_plot,'mo');
ylabel('Pos');
%legend('original','gauss-noise measurement','estimate','vel integral','Location','NorthEastOutside')
legend('original','gauss-noise measurement','estimate');
subplot(3,1,2);
%plot(xt,x_original(2,:),xt,z(2,:),xt,x(2,:),xt(1:499),v);
plot(xt,x_original(2,:),xt,z(2,:),xt,x(2,:));
ylabel('Vel');
%legend('original','gauss-noise measurement','estimate','pos derivative','Location','NorthEastOutside')
legend('original','gauss-noise measurement','estimate');
%subplot(4,1,3);
%plot(xt,x_original(3,:),xt,z(3,:),xt,x(3,:));
%ylabel('Acc');
subplot(3,1,3);
plot(xt,u*F_x);
ylabel('Force input [N]');
