clear all
close all

%%
A=[0 1;0 0];
B=[0;1];
C=[1 0];
D=0;

%syms dt
ts = 5;

sys_c = ss(A,B,C,D)
sys_d = c2d(sys_c,ts)