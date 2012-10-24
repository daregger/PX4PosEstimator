function [x_aposteriori,P_aposteriori]  = positionKalmanfilter(dt,udpIndVect,z_k,u,Rot_matrix,x_aposteriori_k,P_aposteriori_k,knownConstPos)



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




Fu=



ax=sin(theta)*Fu/m;
ay=sin(phi)*Fu/m;
az=Rot_matrix(3,3)*Fu/m;


z=[xdd;ydd;zdd];
%%
% |x''' |           |0,   0,  0,  0, 0, 0, 0, 0, 0, 0;|  |x'' |   |0   0   0  |
% |x''  |           |1,   0,  0,  0, 0, 0, 0, 0, 0, 0;|  |x'  |   |1/m 0   0  |
% |x'   |           |0,   1,  0,  0, 0, 0, 0, 0, 0, 0;|  |x   |   |0   0   0  |
% |y''' |           |0,   0,  0,  0, 0, 0, 0, 0, 0, 0;|  |y'' |   |0   0   0  |
% |y''  |           |0,   0,  0,  1, 0, 0, 0, 0, 0, 0;|  |y'  |   |0   1/m 0  |
% |y'  | =          |0,   0,  0,  0, 1, 0, 0, 0, 0, 0;|* |y   | + |0   0   0  |
% |z''' |           |0,   0,  0,  0, 0, 0, 0, 0, 0, 0;|  |z'' |   |0   0   0  |
% |z''  |           |0,   0,  0,  0, 0, 0, 1, 0, 0, 0;|  |z'  |   |0   0   1/m|
% |z'   |           |0,   0,  0,  0, 0, 0, 0, 1, 0, 0;|  |z   |   |0   0   0  |
% |agl'|            |0,   0,  0,  0, 0, 0, 0, 0, 0, 0;|  |agl |   |0   0   0  |
%                  A                 * x      +     B*u

% |px   |           |0,   0,  1,  0, 0, 0, 0, 0, 0, 0;|  |x'' |  
% |py   |           |0,   0,  0,  0, 0, 1, 0, 0, 0, 0;|  |x'  |  
% |pz   |           |0,   0,  0,  0, 0, 0, 0, 0, 1, 0;|  |x   |  
% |p2x  |           |0,   0,  1,  0, 0, 0, 0, 0, 0, 0;|  |y'' |  
% |p2y  |           |0,   0,  0,  0, 0, 1, 0, 0, 0, 0;|  |y'  | 
% |p2z  | =         |0,   0,  0,  0, 0, 0, 0, 0, 1, 0;|* |y   | 
% |vx   |           |0,   1,  0,  0, 0, 0, 0, 0, 0, 0;|  |z'' |  
% |vy   |           |0,   0,  0,  0, 1, 0, 0, 0, 0, 0;|  |z'  |   
% |vz   |           |0,   0,  0,  0, 0, 0, 0, 1, 0, 0;|  |z   |   
% |Ax   |           |1,   0,  0,  0, 0, 0, 0, 0, 0, 0;|  |z   |   
% |Ay   |           |0,   0,  0,  1, 0, 0, 0, 0, 0, 0;|  |z   |   
% |Az   |           |0,   0,  0,  0, 0, 0, 1, 0, 0, 0;|  |z   |   
% | baro|           |0,   0,  0,  0, 0, 0, 0, 0, 1, 0;|  |agl |   
% | son |  =        |0,   0,  0,  0, 0, 0, 0, 0, 0, 1;| 
%   y   =                       C                 *     x     

tpred=0.005;
m=0.446;
A=[   0,   0,  0,  0, 0, 0, 0, 0, 0, 0; 
      1,   0,  0,  0, 0, 0, 0, 0, 0, 0;
      0,   1,  0,  0, 0, 0, 0, 0, 0, 0;  
      0,   0,  0,  0, 0, 0, 0, 0, 0, 0;  
      0,   0,  0,  1, 0, 0, 0, 0, 0, 0;  
      0,   0,  0,  0, 1, 0, 0, 0, 0, 0;
      0,   0,  0,  0, 0, 0, 0, 0, 0, 0;  
      0,   0,  0,  0, 0, 0, 1, 0, 0, 0;  
      0,   0,  0,  0, 0, 0, 0, 1, 0, 0;  
      0,   0,  0,  0, 0, 0, 0, 0, 0, 0] 


b=[0,   0,   0;  
  1, 0,   0; 
  0,   0,   0;  
  0,   0,   0;  
  0,   1, 0;  
  0,   0,   0;  
  0,   0,   0;  
  0,   0,   1;
  0,   0,   0;  
  0,   0,   0];  


C=[0,   0,  1,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 1, 0, 0, 0, 0;  
0,   0,  0,  0, 0, 0, 0, 0, 1, 0;
0,   0,  1,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 1, 0, 0, 0, 0;
0,   0,  0,  0, 0, 0, 0, 0, 1, 0;
0,   1,  0,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 1, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 0, 0, 1, 0, 0;  
1,   0,  0,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  1, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 0, 1, 0, 0, 0;
0,   0,  0,  0, 0, 0, 0, 0, 1, 0; 
0,   0,  0,  0, 0, 0, 0, 0, 0, 1]
D=[];


sys=ss(A,b,C,D);

sysd=c2d(sys,tpred);


F=sysd.a;

B=sysd.b;

H=sysd.c;
Q = [   diag(knownConstPos(1:3)),   zeros(3),                   zeros(3),                       0;
        zeros(3),                   diag(knownConstPos(4:6)),   zeros(3),                       0;
        zeros(3),                   zeros(3),                   diag(knownConstPos(5:9)),       0;
        0,                          0,                          0,                              knownConstPos(10)]   ;                  

Q = [   diag(knownConstPos(11:13)),   zeros(3),                     zeros(3),                           0;
        zeros(3),                   diag(knownConstPos(14:16)),     zeros(3),                           0;
        zeros(3),                   zeros(3),                       diag(knownConstPos(15:19)),         0;
        0,                          0,                              0,                                  knownConstPos(20)]   ; 


 F =    [1.0000,    0 ,        0,       0,          0,          0,          0,          0,          0,          0;
         dt,        1.0000,    0,       0,          0,          0,          0,          0,          0,          0;
         0.5*dt^2,  dt,        1.0000,  0,          0,          0,          0,          0,          0,          0;
         0,         0,         0,       1.0000,     0,          0,          0,          0,          0,          0;
         0,         0,         0,       dt,         1.0000,     0,          0,          0,          0,          0;
         0,         0,         0,       0.5*dt^2,   dt,         1.0000,     0,          0,          0,          0;
         0,         0,         0,       0,          0,          dt,         1.0000,     0,          0,          0;
         0,         0,         0,       0,          0,          0.5*dt^2,   dt,         1.0000,     0,          0;
         0,         0,         0,       0,          0,          0,          0,          0,          1.0000,     0;
         0,         0,         0,       0,          0,          0,          0,          0,          0,          1.0000];
     
     

 B = [   0,             0,         0;
         dt,            0,         0;
         0.5*dt^2,      0,         0;
         0,             0,         0;
         0,             dt,        0;
         0,             0.5*dt^2,  0;
         0,             0,         0;
         0,             0,         dt;
         0,             0,         0.5*dt^2;
         0,             0,         0];

H_full=[0,   0,  1,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 1, 0, 0, 0, 0;  
0,   0,  0,  0, 0, 0, 0, 0, 1, 0;
0,   0,  1,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 1, 0, 0, 0, 0;
0,   0,  0,  0, 0, 0, 0, 0, 1, 0;
0,   1,  0,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 1, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 0, 0, 1, 0, 0;  
1,   0,  0,  0, 0, 0, 0, 0, 0, 0;
0,   0,  0,  1, 0, 0, 0, 0, 0, 0;
0,   0,  0,  0, 0, 0, 1, 0, 0, 0;
0,   0,  0,  0, 0, 0, 0, 0, 1, 0; 
0,   0,  0,  0, 0, 0, 0, 0, 0, 1];
    
%%prediction step
x_apriori=F*x_aposteriori_k+B*Fu(1:3);
P_apriori=F*P_aposteriori_k*F'+Q; 



if ~isempty(udpIndVect)==1
    H= H_full(udpIndVect,:);
    %%update step

    y_k=z_k-H_k*x_apriori;
    
    %measurement noise covariance matrix
  
    S_k=H_k*P_apriori*H_k'+R(udpIndVect,udpIndVect);
    K_k=(P_apriori*H_k'/(S_k));
    
    
    x_aposteriori=x_apriori+K_k*y_k;
    P_aposteriori=(eye(10)-K_k*H_k)*P_apriori;
else
    x_aposteriori=x_apriori;
    P_aposteriori=P_apriori;
end











