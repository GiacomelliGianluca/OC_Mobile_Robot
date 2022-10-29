function zsim = r_simulation(z0,u,d,th,N_FFD,Ts_FFD,theta_dot)

zsim = zeros(7,Nsim+1);
zsim = z0;

for ind = 1:Nsim
    zsim(:,ind+1) = zsim(:,ind) + Ts*robot_dyn_model(t,z,u,d,th,theta_dot);
    theta_FFD(1,ind)    =      zout_FFD(3,ind);                                                                 % update of the yaw angle                                               
    theta_dot           =      (theta_FFD(1,ind)-theta_FFD(1,ind-1))/Ts_FFD;

end
