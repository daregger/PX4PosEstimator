syms cv dT st
A=[1,dT,st;
    0,1,dT;
    0,0,1];
Q=A*[0,0,0;0,0,0;0,0,cv]*A'