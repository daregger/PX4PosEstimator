function [x_aposteriori,P_aposteriori]  = positionKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k)



%Extended Attitude Kalmanfilter
%
%state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]'
%measurement vector z has the following entries [xVicon,yVicon,zVicon||xVision,yVision,zVision||xVel,yVel,zVel||xAcc,yAcc,zAcc||zbaro||zrel]'
%knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW]
%
%[x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
%
%Example....
%
% $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 


%%define the matrices







dt=6.9367e-003;

% A=[   0,   0,  0; 
%       1,   0,  0;
%       0,   1,  0];  
%      
% 
% b=[0;
%     0;
%     0];  
% 
% 
% C=[1,0,0];
% D=[];
% 
% 
% sys=ss(A,b,C,D);
% 
% sysd=c2d(sys,tpred);
% 
% 
% F=sysd.a;
% 
% B=sysd.b;
% 
% H=sysd.c;
% 
% 

 F =    [1.0000,    0 ,        0      ;
         dt,        1.0000,    0      ;
         0.5*dt^2,  dt,        1.0000];
     
     

 B = [  0;0;0];

H_full=[1,0,0];
    
%%prediction step
Q=[0.5,0,0;
    0,0.0001,0;
    0,0,0.0001];
R=0.001;
x_apriori=F*x_aposteriori_k;
P_apriori=F*P_aposteriori_k*F'+Q; 




    H_k= H_full;
    %%update step

    y_k=z_k-H_k*x_apriori;
    
    %measurement noise covariance matrix
  
    S_k=H_k*P_apriori*H_k'+R;
    K_k=(P_apriori*H_k'/(S_k));
    
    
    x_aposteriori=x_apriori+K_k*y_k;
    P_aposteriori=(eye(3)-K_k*H_k)*P_apriori;

end











