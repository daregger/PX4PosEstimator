%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INVPENDKALMAN - Kalman filter with state feedback example for RE 2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Class:
% Recursive Estimation
% Spring 2012
% Recitation Kalman Filtering - Linear Time Invariant
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Raffaello D'Andrea, Philipp Reist
% reistp@ethz.ch
%
% --
% Revision history
% [30.04.12, PR]    first version

function TIKalmanEx_2012()
% Introduction:
% In this file, we design a linear time-invariant LQR controller to stabilize
% an inverted pendulum at the upright position. The LQR controller is a
% state feedback controller: In order to calculate the control input,
% you need access to the full state of the system. Usually however, you
% only have access to a few measurements, for example here,
% the position and angle of the cart and pendulum, respectively.
% Therefore, you need to reconstruct the full state from the limited
% measurements. There are two obvious options: a) "only" measurements: you
% simply differentiate the measured positions and angles to obtain
% velocities. Differentiating greatly amplifies mesurement noise, so
% therefore you most likely will have to design a low-pass filter for
% the velocities before using them to calculate the control signal
% (avoid spikes in control input). 2) measurements plus knowledge of
% system dynamics: Design a Kalman filter. With the additional
% knowledge of the system dynamics you expect this state estimate to be
% superior to the purely data driven approach, as long as your system
% model is reasonably good. You can tweak the Kalman filter with the
% design parameters Q and R, i.e. process and measurement noise
% variances, to trade-off model vs. measurement quality.
%
% Below, we design a Kalman filter for the inverted pendulum and
% compare the resulting state estimate to the ground truth measured in
% simulation. Play around with the different parameters to learn more
% about tuning the filter and learn about the effects of Q and R.
%
%% The inverted pendulum system, or cart-pole system:
% We consider the control of the following system:
%
%                         O  Pendulum, mass mp
%                        / length L
%    input force F   ___/__
%               --->| cart | mass mc
% __________________0------0__________________________
%
% an actuated cart with a non-actuated pendulum attached.
%
% Cart position (m):    c, zero at center of rail
% Pendulum angle (rad): p, zero at lower stable equilibrium, pi at upper
%                       unstable equilibrium, positive
%                       counter-clockwise
%
% The continuous system dynamics are given in the dynamics function.
% The state is: x = [c, p, dc/dt, dp/dt]
% The input is: u = F, the force acting on the cart in positive c
% direction

% System parameters (Yes, the same you used in Digital Control Systems)
mc = 1.5;   % cart mass
mp = 0.175; % pendulum mass
g = 9.81;   % gravitational acceleration constant
l = 0.28;   % pendulum length

% Goal state where we will stabilize the system:
% pendulum up and cart at rest at center of rail
xg = [0,pi,0,0]'; % 4x1 vector
ug = 0;     % goal state input, zero since xg is equilbrium

% Discrete-time measurments: c[k] = c(t = k*T), T: sampling time
% zc[k] = c[k] + wc[k]: noisy position measurement
% wc is white noise, uniformly distributed on the interval [-Wc, Wc]
% zp[k] = p[k] + wp[k]: noisy angle measurement
% wp is white noise, uniformly distributed on the interval [-Wp, Wp]
Wc = 0.01;        % position noise bound, meters
Wp = 0.5*pi/180;  % angle noise bound, radians
% measurement z[k] = [zc[k];zp[k]], 2x1 vector
% noise       w[k] = [wc[k];wp[k]], 2x1 vector
% you might have identified these noise statistics by measuring the
% output for a still cart an pendulum and inspecting the measurement
% statistics

%% Linearized Dynamics at equilibrium:
% define xb = x-xg, the deviation from the goal state
% since u - ug = u, we do not introduce ub
% linearize the system:
% f(xg + xb) = f(xg) + df/dx(xg,ug) xb + df/du(xg,ug) u
% get Jacobian matrices Ac and Bc (continuous time):
[~,df] = dynamics(0,xg,ug);
Ac = df.A;  % = df/dx(xg,ug)
Bc = df.B;  % = df/du(xg,ug)

% measurement equation:
% z = H*x
H = zeros(2,4);
H(1,1) = 1;
H(2,2) = 1; % we only have access to the position and angle

% Discrete-time system dynamics:
T = 0.05;   % sampling time
% exact discretization:
AB = [Ac, Bc; zeros(1,length([xg;ug]))]; % construct dummy matrix
ABd = expm(AB*T);                        % matrix exponential
Ad = ABd(1:length(xg), 1:length(xg));    % Discrete-time A
Bd = ABd(1:length(xg),length(xg)+1);     % Discrete-time B
% note that Hd = H

% the full, linear, time invariant, discrete-time model is now:
% xb[k] = Ad * xb[k-1] + Bd*u[k] + Bd*v[k]
% z[k]  = H  * xb[k]   + w[k]
% the process noise v[k] is modeled as noise on the actual force
% exerted on the cart and is white with a uniform distribution with
% bounds [-V,V]
V = 1; % (N)

% LQR controller design:
Kc = dlqr(Ad,Bd,diag([1,1,0.1,0.1]),1);
% given the estimate xbh = xh - xg (h = hat), the control input is
% calculated as: u[k] = -Kc*xbh[k-1]

%% Kalman Filter design:
% variance of measurements:
sigmac2 = 1/3*Wc^2;  % variance of a uniform distribution with zero mean (m^2)
sigmap2 = 1/3*Wp^2;  % dito (rad^2)

% TUNING PARAMETERS (PLAY AROUND WITH THESE!)
% The current values are what they should be based on the variances used in
% the model. In a real design you would not have such precise noise
% statistics available and you would tune R and Q to get the desired
% filter performance.
R = diag([sigmac2, sigmap2]);
Q = 1/3*Bd*V^2*Bd'; % need to consider Bd for variance of process
                    % noise signal Bd*v[k], (linear coordinate
                    % transformation).

% get time-invariant variance Pinf by solving dare:
% Pinf = Ad*Pinf*Ad' - Ad*Pinf*H'*(H*Pinf*H'+R)^-1*H*Pinf*Ad'+Q
Pinf = dare(Ad',H',Q,R);
% why Ad', H'? Because the dare function solves:
% Pinf = Ad'*Pinf*Ad - Ad'*Pinf*H*(H'*Pinf*H+R)^-1*H'*Pinf*Ad + Q

% calculate the Kalman gain:
Kf = Pinf*H'/(H*Pinf*H'+R);

%% Let's do the actual filtering and simulation of the system:
% initialization of system:
xb = [0.2,0.1,0,0]';  % initial condition for nonlinear system

% initial estimator state
xbh = zeros(4,1);   % initial mean is 0, play with this value

% Simulation options:
USE_ESTIMATE_CTRL = 0; % if 1, the controller uses the estimate
USE_PROC_NOISE = 0;    % if 1, the simulation includes process noise
USE_MEAS_NOISE = 0;    % if 1, the simulation includes measurement noise
USE_NONLINEAR_DYN = 1; % if 1, the simulation runs with the nonlinear system
                       % dynamics
                       
Nsteps = 150; % number of simulation steps:
% init storage matrices:
us = [0,zeros(1, Nsteps)];                      % input history
xbs = [xb, zeros(length(xb),Nsteps)];           % actual state deviation
xbhs = [xbh,zeros(length(xb),Nsteps)];          % estimator state

for k = 2:(Nsteps+1)
    % current control input u[k] = -K*xbh[k-1]:
    % switch between estimated state for control and actual state:
    if(USE_ESTIMATE_CTRL)
        u = -Kc*xbh; % use estimator value
    else
        u = -Kc*xb; % use true value (use to see Kalman filter behavior)
    end

    
    % prediction step / step 1:    
    xbp = Ad*xbh + Bd*u; % simply the last mean pushed forward by dynamics
    
    % add input noise for simulation if switch set accordingly:
    if(USE_PROC_NOISE)
        unoise = u - V + 2*V*rand;
    else
        unoise = u;
    end
    
    if(USE_NONLINEAR_DYN)
        % update true model by simulating nonlinear model (need to add
        % nominal state again to recover correct dynamics)
        [~,xsim] = ode45(@(t,x)dynamics(t,x,unoise),[0 T], (xb + xg));
        xb = xsim(end,:)' -  xg;   % update state deviation after sim
    else
        % use approximated discrete time dynamics
        xb = Ad*xb + Bd*unoise;
    end
    
    % generate noisy measurement if switch set accordingly:
    if(USE_MEAS_NOISE)
        z = H*xb + [-Wc + 2*Wc*rand; -Wp + 2*Wp*rand];
    else
        z = H*xb;
    end
    
    % alternatively, you could do one-step update:
    % xbh = (eye(length(xbh)) - Kf*H)*(Ad*xbh + Bd*u) + Kf*z;
    
    % measurement update / step 2:
    xbh = xbp + Kf*(z - H*xbp);
    
    % store everything in history:
    us(:,k) = u;
    xbs(:,k) = xb;
    xbhs(:,k) = xbh;
end

%% plot all

% switch plotting one std deviation on or off:
PLOT_STD_DEV = 0;

% plot results:
tm = 0:Nsteps;
figure(1)
subplot(4,1,1)
plot(tm,xbs(1,:) + xg(1),'LineWidth',2), hold on
plot(tm,xbhs(1,:) + xg(1),'r','LineWidth',2)
if(PLOT_STD_DEV)
    plot(tm,xbhs(1,:) + sqrt(Pinf(1,1)) + xg(1),'g')
    plot(tm,xbhs(1,:) - sqrt(Pinf(1,1)) + xg(1),'g')
end
hold off
ax = axis;
axis([ax(1:2), [-0.2,0.2]]);
xlabel('Step k')
ylabel('Cart Position c (m)')
if(PLOT_STD_DEV)
legend('True','Estimate', '1 std dev of Pinf')
else
legend('True','Estimate')
end    
subplot(4,1,2)
plot(tm,xbs(2,:) + xg(2),'LineWidth',2), hold on
plot(tm,xbhs(2,:) + xg(2),'r','LineWidth',2)
if(PLOT_STD_DEV)
    plot(tm,xbhs(2,:) + sqrt(Pinf(2,2)) + xg(2),'g')
    plot(tm,xbhs(2,:) - sqrt(Pinf(2,2)) + xg(2),'g')
end
hold off
ax = axis;
axis([ax(1:2), [pi-0.1,pi+0.1]]);
xlabel('Step k')
ylabel('Pendulum Angle p (rad)')
if(PLOT_STD_DEV)
legend('True','Estimate', '1 std dev of Pinf')
else
legend('True','Estimate')
end    

subplot(4,1,3)
plot(tm,xbs(3,:) + xg(3),'LineWidth',2), hold on
plot(tm,xbhs(3,:) + xg(3),'r','LineWidth',2)
if(PLOT_STD_DEV)
    plot(tm,xbhs(3,:) + sqrt(Pinf(3,3))+ xg(3),'g')
    plot(tm,xbhs(3,:) - sqrt(Pinf(3,3))+ xg(3),'g')
end
hold off
ax = axis;
axis([ax(1:2), [-0.2,0.2]]);
xlabel('Step k')
ylabel('Cart Velocity dc/dt (m/s)')
if(PLOT_STD_DEV)
legend('True','Estimate', '1 std dev of Pinf')
else
legend('True','Estimate')
end    

subplot(4,1,4)
plot(tm,xbs(4,:) + xg(4),'LineWidth',2), hold on
plot(tm,xbhs(4,:) + xg(4),'r','LineWidth',2)
if(PLOT_STD_DEV)
    plot(tm,xbhs(4,:) + sqrt(Pinf(4,4)) + xg(4),'g')
    plot(tm,xbhs(4,:) - sqrt(Pinf(4,4)) + xg(4),'g')
end
hold off
ax = axis;
axis([ax(1:2), [-0.4,0.4]]);
xlabel('Step k')
ylabel('Pendulum Velocity dp/dt (rad/s)')
if(PLOT_STD_DEV)
legend('True','Estimate', '1 std dev of Pinf')
else
legend('True','Estimate')
end    

    function [xdot, df] = dynamics(t,x,u)
        % ==================================================================
        % This function defines the continuous dynamics of the inv. pendulum
        % ==================================================================
        % x = (c,p,dc/dt,dp/dt), pendulum upright at rest when
        % x = (0,pi,0,0)
        xdot=[x(3),x(4),(mc+mp+(-1).*mp.*cos(x(2)).^2).^(-1).*(u+mp.*(l.*x(4).^2+ ...
            g.*cos(x(2))).*sin(x(2))),(-1).*l.^(-1).*(mc+mp+(-1).*mp.* ...
            cos(x(2)).^2).^(-1).*(g.*(mc+mp).*sin(x(2))+cos(x(2)).*(u+l.* ...
            mp.*x(4).^2.*sin(x(2))))]';
        
        % also output the jacobian matrices A and B if more than one output
        % argument is requested
        if (nargout>1)
            df.A=[0,0,1,0;0,0,0,1;0,mp.*(2.*mc+mp+(-1).*mp.*cos(2.*x(2))).^(-2) ...
                .*((-2).*g.*mp+l.*(4.*mc+(-1).*mp).*x(4).^2.*cos(x(2))+2.* ...
                g.*(2.*mc+mp).*cos(2.*x(2))+l.*mp.*x(4).^2.*cos(3.*x(2))+(-4) ...
                .*u.*sin(2.*x(2))),0,2.*l.*mp.*x(4).*(mc+mp+(-1).*mp.*cos(x(2)) ...
                .^2).^(-1).*sin(x(2));0,l.^(-1).*(mc+mp+(-1).*mp.*cos(x(2)).^2) ...
                .^(-2).*((-1/2).*(2.*mc+mp+(-1).*mp.*cos(2.*x(2))).*(g.*( ...
                mc+mp).*cos(x(2))+l.*mp.*x(4).^2.*cos(2.*x(2))+(-1).*u.*sin(x(2)))+ ...
                2.*mp.*cos(x(2)).*sin(x(2)).*(g.*(mc+mp).*sin(x(2))+cos(x(2)).*( ...
                u+l.*mp.*x(4).^2.*sin(x(2))))),0,(-2).*mp.*x(4).*cos(x(2)).*(mc+mp+( ...
                -1).*mp.*cos(x(2)).^2).^(-1).*sin(x(2))];
            df.B=[0,0,(mc+mp+(-1).*mp.*cos(x(2)).^2).^(-1),(-1).*l.^(-1).*cos( ...
                x(2)).*(mc+mp+(-1).*mp.*cos(x(2)).^2).^(-1)]';
        end
    end
end