% Constrained Numerical Optimization for Estimation and Control

%% Initial commands
%clear all
close all
clc

%% Model parameters
Ic      =       1;     
Iw      =       0.1;    
d       =       0.05;      
mw      =       0.1;      
mc      =       1;      
Im      =       0.1;    
R       =       0.025;      
L       =       0.05;     


th      =       [Ic;Iw;d;mw;mc;Im;R;L];


%% Simulation: Initial state
xa          =       0;  
ya          =       0; 
theta       =       0;   
phir        =       0;    
phil        =       0;   
phir_dot    =       0;      
phil_dot    =       0;

z0 = [xa;ya;theta;phir;phil;phir_dot;phil_dot];



%% Simulation with forward finite differences
% Time integration parameters
Ts_FFD      =       1e-2;               % sampling time (s)
Tend_FFD    =       20;                % final time / simulation time (s)
tvec_FFD    =       0:Ts_FFD:Tend_FFD;  % time vector (s)

% Initialize simulation output
N_FFD               =       length(tvec_FFD);   % number of samples
zout_FFD            =       zeros(7,N_FFD);     % matrix with states
zout_FFD(:,1)       =       z0;                 % matrix of the states z(t+1) 


%% Simulation: step amplitude
% Input variables:
taur        =       zeros(1,N_FFD);          %       taul = left-wheel torque  
taul        =       zeros(1,N_FFD);          %       taul = left-wheel torque

for ind=1:N_FFD
    if ind <= round(N_FFD/4)
        taur(ind) = 0.5;
        taul(ind) = 0.5;
    elseif ind > round(N_FFD/4) && ind <= round(N_FFD/2)
        taur(ind) = 0.5;
        taul(ind) = 0;
    elseif ind > round(N_FFD/2) % && ind <= round(3*N_FFD/4)
        taur(ind) = 0;
        taul(ind) = 0.5;
%     elseif ind > round(9/10*N_FFD) 
%         taur(ind) = -2;
%         taul(ind) = -2;
    end
end

uout_FFD            =      zeros(2,N_FFD);     % matrix with inputs
u                   =      [taur; taul];

uout_FFD            =          u;                  % matrix of the inputs


%% Run simulation

% Computation of zout_FFD using Forward Euler Method
tic
for ind=2:N_FFD
    zdot                =      robot_dyn_model(0,zout_FFD(:,ind-1),uout_FFD(:,ind-1),0,th);          % derivatives of the state variables
    zout_FFD(:,ind)     =      zout_FFD(:,ind-1)+Ts_FFD*zdot;                                                  % update t given t-1                                   
    %uout_FFD(:,ind)     =      uout_FFD(:,1);
%     if (ind >= round(N_FFD/2))
%         uout_FFD(:,ind) = [5 0]';  
%     end
end
t_FFD = toc

% Plot the results
figure(1),p1_FFD = plot(zout_FFD(1,:),zout_FFD(2,:), 'linewidth', 2);grid on, hold on,xlabel('X (m)'),ylabel('Y (m)'), title('Trajectory YX'), hold on
figure(2),p2_FFD = plot(tvec_FFD,zout_FFD(3,:), 'linewidth', 2);grid on, hold on,xlabel('Time (s)'),ylabel('Yaw angle (rad)'), title('Yaw vs time'), hold on
figure(3),p3_FFD = plot(tvec_FFD,zout_FFD(6,:), 'linewidth', 2);grid on, hold on,xlabel('Time (s)'),ylabel('Righ wheel speed (rad/s)'), title('Right wheel speed'), hold on
figure(4),p4_FFD = plot(tvec_FFD,zout_FFD(7,:), 'linewidth', 2);grid on, hold on,xlabel('Time (s)'),ylabel('Left wheel speed (rad/s)'), title('Left wheel speed'), hold on


%% Visualizer
% 
% X = zout_FFD(1,:)';
% Y = zout_FFD(2,:)';
% THETA = zout_FFD(3,:)';
% 
% V = [X Y THETA];
% robot = struct;
% robot.image = imread("robot.jpg");
% robot.rotation = 0;
% robot.centre = [500,500];
% robot.length = 0.25;
% 
% 
% figure(1)
% h = plot_vehicle(V,'fps', 60, 'model', robot, '--')

%% % %% Simulation with ode45
% 
% % Time integration parameters
% Ts_o45      =       0.1;               % sampling time (s)
% Tend_o45    =       100;                % final time (s)
% tvec_o45    =       0:Ts_o45:Tend_o45;  % time vector (s)
% 
% % Initialize simulation output
% N_o45               =       length(tvec_o45);   % number of samples
% zout_o45            =       zeros(7,N_o45);     % matrix with states
% uout_o45            =       zeros(2,N_o45);     % matrix with inputs
% zout_o45(:,1)       =       z0;
% uout_o45(:,1)       =       u;
% theta_o45           =       zeros(1,N_o45);
% 
% % Run simulation
% tic
% theta_dot = 0;
% for ind=2:N_o45
%     zout_temp           =   ode45(@(t,z)robot_dyn_model(t,z,uout_o45(:,ind-1),0,th),[0 Ts_o45],zout_o45(:,ind-1));
%     zout_o45(:,ind)     =   zout_temp.y(:,end);
%     %uout_o45(:,ind)     =   uout_o45(:,1);
% %     if (ind >= round(N_o45/2))
% %         uout_o45(:,ind) = [5 0]';  
% %     end
% end
% t_o45 = toc
% 
%
% 
% % Plot the results
% figure(1),p1_o45 = plot(zout_o45(1,:),zout_o45(2,:));grid on, hold on,xlabel('X (m)'),ylabel('Y (m)'), title('Trajectory YX')
% figure(2),p2_o45 = plot(tvec_o45,zout_o45(3,:));grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)'), title('Yaw vs time')
% figure(3),p3_o45 = plot(tvec_o45,zout_o45(6,:));grid on, hold on,xlabel('Time (s)'),ylabel('Righ wheel speed (rad/s)'), title('Right wheel speed')
% figure(4),p4_o45 = plot(tvec_o45,zout_o45(7,:));grid on, hold on,xlabel('Time (s)'),ylabel('Left wheel speed (rad/s)'), title('Left wheel speed')
% 
% h1 = [p1_FFD p1_o45]';
% h2 = [p2_FFD p2_o45]';
% h3 = [p3_FFD p3_o45]';
% h4 = [p4_FFD p4_o45]';
% 
% legend(h1, 'FFD', 'o45');
% legend(h2, 'FFD', 'o45');
% legend(h3, 'FFD', 'o45');
% legend(h4, 'FFD', 'o45');
% 
