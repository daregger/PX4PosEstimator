%%  Kalman Filter iteration
for k = 1:stop-1
    %Step 1 prediction/a priori
    x_apriori = A*x(:,k)+B*u(k+1);                          %project state ahead
    P_apriori = A*P*A'+Q;                                 %project error covariance ahead
    %Step 2 correction/a posteriori
    if pos_mask(1,k+1) == 1
        K = P_apriori*H'*inv(H*P_apriori*H+R);                      %compute kalman gain
        x(:,k+1) = x_apriori+K*(z(:,k+1)-H*x_apriori);          %update estimate via z
        P = (eye(2)-K*H)*P_apriori;                              %update error covariance
    else
       x(:,k+1)=x_apriori;
       P=P_apriori;
    end        
end