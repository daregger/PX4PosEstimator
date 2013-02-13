clear all
close all
% to tune
q = 1000;
r = 0.005;

%
sensorsAll = importdata('shortened.mat');
start_sim = 1;
end_sim = length(sensorsAll(:,51));
dT_onboard_cum = (sensorsAll(1:end_sim,1)).*1e-6;
dT_onboard = zeros(end_sim,1);
for i=start_sim+1:end_sim
    dT_onboard(i,1)=dT_onboard_cum(i,1)-dT_onboard_cum(i-1,1);
end
%plot(dT_onboard)
f_mean = 1/mean(dT_onboard);

% sensor readout
x = zeros(end_sim,1);
y = zeros(end_sim,1);
x(start_sim:end_sim) = sensorsAll(start_sim:end_sim,51);
y(start_sim:end_sim) = sensorsAll(start_sim:end_sim,52);%31
scale = 10;
vx_est = zeros(end_sim,1);
vy_est = zeros(end_sim,1);
vx_est(start_sim:end_sim) = scale*sensorsAll(start_sim:end_sim,54);
vy_est(start_sim:end_sim) = scale*sensorsAll(start_sim:end_sim,55);

% simulation
q_start = 1;
q = q_start;
q_temp = 1;
q_it = 7;
simPlotValx = zeros(end_sim,q_it);
simPlotValvx = zeros(end_sim,q_it);
while q_temp <= q_it
    %sim
        % dlqe
        dT = 1/f_mean;
        A = [1 dT;
             0 1];
        G = [1 0;
            0 1]; 
        C = [1 0];
        Qtemp = [0 0;
                  0 q];
        Q = A*Qtemp*A';
        R = r;
        [M,P,Z,E] = dlqe(A,G,C,Q,R);
        % init
        gain_k1 = M(1)
        gain_k2 = M(2)

        simPlotValx(1,q_temp) = 0;
        simPlotValvx(1,q_temp) = 0;
        for i = 1:end_sim-1
            % pos x, vel x
            simPlotValx(i+1,q_temp) = simPlotValx(i,q_temp)+dT*simPlotValvx(i,q_temp) + gain_k1*(x(i) - (dT*simPlotValvx(i,q_temp)+simPlotValx(i,q_temp)));
            simPlotValvx(i+1,q_temp) = simPlotValvx(i,q_temp) + gain_k2*(x(i) - simPlotValvx(i,q_temp));
        end
    %end sim
    q = q*10;
    q_temp = q_temp + 1;
end

% discrete derivation of vicon pos for vicon vel
% first low pass
windowSize = 30;
%plot(x)
vx_der(:,1) = filter(ones(1,windowSize)/windowSize,1,x(:));
%plot(vx_der)
% then discrete derivate v=dx/dt
vx_der(:,1) = [diff(vx_der(:,1)) ;0];  
for i = start_sim:end_sim-1
    %i
    vx_der(i,1) = vx_der(i,1)/dT_onboard(i,1);  
end

figure('units','normalized','outerposition',[0 0 1 1])

subplot(2,1,1)
plot(x,'g');
hold on
plot(vx_der,'b');
plot(simPlotValx(:,1),'y');
plot(simPlotValvx(:,1),'c')
legend('x','vx\_derivative','x_dlqe','v_dlqe');

subplot(2,1,2)
plot(x,'g');
hold on
plot(vx_der,'b');
legend('x','vx\_derivative')

%% table gen for 50 Hz
q=[0.01 0.1 1 10 100 1000];
k1 = [0.2 0.34 0.5 0.7 0.9 0.98];
k2 = [1.25 3.6 9.6 22 39 48];
%plot(q,k1,q,k2)

