clear all
close all

sensorsAll = importdata('shortened.mat');

start_sim = 1;
end_sim = length(sensorsAll(:,51))

dT_onboard_cum = (sensorsAll(1:end_sim,1)).*1e-6;
dT_onboard = zeros(end_sim,1);
for i=start_sim+1:end_sim
    dT_onboard(i,1)=dT_onboard_cum(i,1)-dT_onboard_cum(i-1,1);
end
%plot(dT_onboard)
f_mean = 1/mean(dT_onboard);

x = zeros(end_sim,1);
y = zeros(end_sim,1);
x(start_sim:end_sim) = sensorsAll(start_sim:end_sim,51);
y(start_sim:end_sim) = sensorsAll(start_sim:end_sim,52);%31
scale = 10;
vx_est = zeros(end_sim,1);
vy_est = zeros(end_sim,1);
vx_est(start_sim:end_sim) = scale*sensorsAll(start_sim:end_sim,54);
vy_est(start_sim:end_sim) = scale*sensorsAll(start_sim:end_sim,55);

%% discrete derivation of vicon pos for vicon vel
% first low pass
windowSize = 30;
%plot(x)
vx_der(:,1) = filter(ones(1,windowSize)/windowSize,1,x(:));
%plot(vx_der)

vy_der(:,1) = filter(ones(1,windowSize)/windowSize,1,y(:));
% then discrete derivate v=dx/dt
vx_der(:,1) = [diff(vx_der(:,1)) ;0];  
vy_der(:,1) = [diff(vy_der(:,1)) ;0]; 
for i = start_sim:end_sim-1
    %i
    vx_der(i,1) = vx_der(i,1)/dT_onboard(i,1);
    vy_der(i,1) = vy_der(i,1)/dT_onboard(i,1);    
end

figure('units','normalized','outerposition',[0 0 1 1])

subplot(2,1,1)
plot(x,'g');
hold on
plot(vx_est,'r');
plot(vx_der,'b');
legend('x','vx\_estimate','vx\_derivative');

subplot(2,1,2)
plot(y,'g');
hold on
plot(vy_est,'r');
plot(vy_der,'b');
legend('y','vy\_estimate','vy\_derivative');


