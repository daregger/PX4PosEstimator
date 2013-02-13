clear all
close all

sensorsAll = importdata('shortened.mat');
start_sim = 1;
end_sim = length(sensorsAll(:,51));
dT_onboard_cum = (sensorsAll(1:end_sim,1)).*1e-6;
dT_onboard = zeros(end_sim,1);
for i=start_sim+1:end_sim
    dT_onboard(i,1)=dT_onboard_cum(i,1)-dT_onboard_cum(i-1,1);
end
dt = mean(dT_onboard);
% sensor readout
z = zeros(end_sim,1);
z(start_sim:end_sim) = sensorsAll(start_sim:end_sim,51);
vel_raw=diff(z)/dt;

ampl=2.5;
t=dt:dt:20;
%z=ampl*sin(t);
%z=awgn(z,10);
xp=[];

m=0.5;
st=1/2*dt^2;
A=[1,dt,st;
    0,1,dt;
    0,0,1];
C=[1,0,0];
cv=dt^2*ampl^2;
cv2=1e0;
mp=1e-6;
Q=A*[0,0,0;0,0,0;0,0,cv]*A';
R=mp*eye(1);

x_aposteriori_k=[0;0;0];
K=dlqe(A,eye(3),C,Q,R);

for i=1:length(z)
    [x_aposteriori] = kalman_dlqe1(A,C,K,x_aposteriori_k,z(i));
    x_aposteriori_k = x_aposteriori;
    xp=[xp,x_aposteriori_k];   
end

% discrete derivation of vicon pos for vicon vel
% first low pass
windowSize = 30;
vel_filtered(:,1) = filter(ones(1,windowSize)/windowSize,1,z(:));
% then discrete derivate v=dx/dt
vel_filtered(:,1) = [diff(vel_filtered(:,1)) ;0];  
for i = start_sim:end_sim-1
    vel_filtered(i,1) = vel_filtered(i,1)/dt;  
end

plot(z)
hold on
plot(xp(1,:),'r')
plot(xp(2,:),'g')
plot(vel_raw,'c')
plot(vel_filtered,'b');
plot(xp(3,:),'k')
legend('pos measurement','pos est','vel est','vel\_raw(diskret)','vel\_raw(diskret)filtered','beschl est')
