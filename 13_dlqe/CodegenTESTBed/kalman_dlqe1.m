function [x_aposteriori] = kalman_dlqe1(A,C,K,x_aposteriori,z)
    x_aposteriori=A*x_aposteriori+K*(z-C*A*x_aposteriori);      
end