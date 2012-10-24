function [eulerAngles,Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter_wo(updateVect,dt,z,x_aposteriori_k,P_aposteriori_k,q,r)
%#codegen


% Extended Attitude Kalmanfilter
%
% state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]'
% measurement vector z has the following entries [ax,ay,az||mx,my,mz||wmx,wmy,wmz]'
% knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW]
%
% [x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
%
% Example....
%
% $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 $





%coder.varsize('udpIndVect', [9,1], [1,0])
%udpIndVect=find(updVect);


%process and measurement noise covariance matrix

Q = diag(q.^2*dt);



%observation matrix





wx=  x_aposteriori_k(1);
wy=  x_aposteriori_k(2);
wz=  x_aposteriori_k(3);

wox=  x_aposteriori_k(4);
woy=  x_aposteriori_k(5);
woz=  x_aposteriori_k(6);

zex=  x_aposteriori_k(7);
zey=  x_aposteriori_k(8);
zez=  x_aposteriori_k(9);

mux=  x_aposteriori_k(10);
muy=  x_aposteriori_k(11);
muz=  x_aposteriori_k(12);


wk =[wx;
     wy;
     wz];

wok =[wox;woy;woz];
O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]';
zek =(eye(3)+O*dt)*[zex;zey;zez];
muk =(eye(3)+O*dt)*[mux;muy;muz];

EZ=[0,zez,-zey;
    -zez,0,zex;
    zey,-zex,0]';
MA=[0,muz,-muy;
    -muz,0,mux;
    zey,-mux,0]';



E=eye(3);
Es=[1,0,0;
    0,1,0;
    0,0,0];
Z=zeros(3);
x_apriori=[wk;wok;zek;muk];

A_lin=[ Z,  Z,  Z,  Z
        Z,  Z,  Z,  Z
        EZ, Z,  O,  Z
        MA, Z,  Z,  O];


A_lin=eye(12)+A_lin*dt;





P_apriori=A_lin*P_aposteriori_k*A_lin'+Q;


%%update
if updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==1
    R=diag(r);
    %observation matrix
   
    H_k=[  E,     Es,      Z,    Z;
        Z,     Z,      E,    Z;
        Z,     Z,      Z,    E];
   
    y_k=z(1:9)-H_k*x_apriori;
   
    S_k=H_k*P_apriori*H_k'+R;
    K_k=(P_apriori*H_k'/(S_k));
   
   
    x_aposteriori=x_apriori+K_k*y_k;
    P_aposteriori=(eye(12)-K_k*H_k)*P_apriori;
else
    if updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==0
        R=diag(r(1:3));
        %observation matrix
       
        H_k=[  E,     Es,      Z,    Z];
       
        y_k=z(1:3)-H_k(1:3,1:12)*x_apriori;
       
        S_k=H_k(1:3,1:12)*P_apriori*H_k(1:3,1:12)'+R(1:3,1:3);
        K_k=(P_apriori*H_k(1:3,1:12)'/(S_k));
       
       
        x_aposteriori=x_apriori+K_k*y_k;
        P_aposteriori=(eye(12)-K_k*H_k(1:3,1:12))*P_apriori;
    else
        if  updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==0
            R=diag(r(1:6));
            %observation matrix
           
            H_k=[  E,     Es,      Z,    Z;
                Z,     Z,      E,    Z];
           
            y_k=z(1:6)-H_k(1:6,1:12)*x_apriori;
           
            S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6);
            K_k=(P_apriori*H_k(1:6,1:12)'/(S_k));
           
           
            x_aposteriori=x_apriori+K_k*y_k;
            P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori;
        else
            if  updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==1
                R=diag([r(1:3);r(7:9)]);
                %observation matrix
               
                H_k=[  E,     Es,      Z,    Z;
                    Z,     Z,      Z,    E];
               
                y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apriori;
               
                S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6);
                K_k=(P_apriori*H_k(1:6,1:12)'/(S_k));
               
               
                x_aposteriori=x_apriori+K_k*y_k;
                P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori;
            else
                x_aposteriori=x_apriori;
                P_aposteriori=P_apriori;
            end
        end
    end
end



%% euler anglels extraction
z_n_b = -x_aposteriori(7:9)./norm(x_aposteriori(7:9));
m_n_b = x_aposteriori(10:12)./norm(x_aposteriori(10:12));

y_n_b=cross(z_n_b,m_n_b);
y_n_b=y_n_b./norm(y_n_b);

x_n_b=(cross(y_n_b,z_n_b));
x_n_b=x_n_b./norm(x_n_b);





Rot_matrix=[x_n_b,y_n_b,z_n_b];



phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3));
theta=-asin(Rot_matrix(1,3));
psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1));
eulerAngles=[phi;theta;psi];






