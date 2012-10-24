

clc
data=[sysvector(:,2),sysvector(:,3),sysvector(:,4),sysvector(:,5),sysvector(:,6),sysvector(:,7),sysvector(:,8),sysvector(:,9),sysvector(:,10)];


  x_aposteriori_k=[0;0;0;0;0;0;0;0;-9.81;0.24;0.045;0.22];


  
  
  P_aposteriori_k=100*eye(12);

dt=[0.01;   %dt gyro update
    0.01;   %dt acc update
    0.01];  %dt mag update

 

            
            
 xPlot=[];

q=[1e1,1e1,1e1,1e-6,1e-6,1e-6,1e-1,1e-1,1e-1,1e-1,1e-1,1e-1];

r=[1e-2,1e-2,1e-2,1e-1,1e-1,1e-1,1e-1,1e-1,1e-1];
  %inputs=[sysvector(:,18)';sysvector(:,19)';sysvector(:,20)';sysvector(:,21)'];
  %rollpitchyaw=[(sysvector(:,14)-mean(sysvector(:,14)))';(sysvector(:,15)-mean(sysvector(:,15)))';(sysvector(:,16)-mean(sysvector(:,16)))';sysvector(:,17)'];
 for iterations=1:length(data)
     %updateFlagVector
     updVect=[1,1,1];
     udpIndVect=find(updVect);

     
     
    %measurements acc,mag and rates 
    z_rat=[data(iterations,1:3)]';
    z_acc=[data(iterations,4:6)]';
    z_mag=[data(iterations,7:9)]';
    
     %u=[rollpitchyaw(1,iterations);rollpitchyaw(2,iterations);rollpitchyaw(3,iterations);rollpitchyaw(4,iterations)];
    
    
    
    z=[z_rat;z_acc;z_mag];


    [eulerAngels,Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(updVect,dt(1),z,x_aposteriori_k,P_aposteriori_k,q,r);
    xPlot=[xPlot,x_aposteriori];
    x_aposteriori_k=x_aposteriori;
    P_aposteriori_k=P_aposteriori;
 end

  
  %plot(data(:,indexvector(7)),'m')
  figure(1)
  subplot(3,1,1)
  plot(data(:,1),'k')
  title('gyro')
  hold on
  plot(xPlot(1,:),'g');
  subplot(3,1,2)
  plot(data(:,2),'k')
    hold on
  plot(xPlot(2,:),'g');
  %plot(diff(xPlot(1,:))*20,'g');
  subplot(3,1,3)
  plot(data(:,3),'k')
    hold on
  plot(xPlot(3,:),'g');

  figure(2)
  
  hold on
   subplot(3,1,1)
  title('offset')
  hold on
  plot(xPlot(4,:),'r');
   subplot(3,1,2)
  hold on
  plot(xPlot(5,:),'g');
   subplot(3,1,3)
  hold on
  plot(xPlot(6,:),'b');
  
  
  
  figure(3)
  subplot(3,1,1)
  plot(data(:,4),'k')
  title('acc')
  hold on
  plot(xPlot(7,:),'r');
  subplot(3,1,2)
  plot(data(:,5),'k')
  hold on
  plot(xPlot(8,:),'g');

  subplot(3,1,3)
  plot(data(:,6),'k')
  hold on
  plot(xPlot(9,:),'b');
 
figure(4)
  
  
  
  subplot(3,1,1)
  plot(data(:,7),'k')
  title('mag')
  hold on
  plot(xPlot(10,:),'g');
  subplot(3,1,2)
  plot(data(:,8),'k')
  hold on
  plot(xPlot(11,:),'g');
  subplot(3,1,3)
  plot(data(:,9),'k')
  hold on
  plot(xPlot(12,:),'b');
  %figure(24)
  
  
  
