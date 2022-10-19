% Constrained Numerical Optimization for Estimation and Control
% Laboratory session A
% Script to initialize and simulate a 6 d.o.f. nonlinear vehicle model
% using different simulation approaches.
% Vehicle parameters taken from Canale et al., "Robust vehicle yaw control 
% using an active differential and IMC techniques", Control Engineering
% Practice 15, pp. 923-941, 2007

%% Initial commands
clear all
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
xa_dot      =       0;  
ya_dot      =       0; 
theta_dot   =       0;   
phir_dot    =       0;    
phil_dot    =       0;  

%phir_dot =       0;      
%phil_dot =       0;

%z0 = [xa;ya;theta;phir;phil;phir_dot;phil_dot];
z0 = [xa;ya;theta;phir;phil;xa_dot;ya_dot;theta_dot;phir_dot;phil_dot];


%% Simulation: step amplitude
% Input variables:
taur        =       0;          %       taul = left-wheel torque  
taul        =       5;          %       taul = left-wheel torque
u = [taur; taul];               %       input vector

%% Simulation with forward finite differences
% Time integration parameters
Ts_FFD      =       1e-1;               % sampling time (s)
Tend_FFD    =       100;                % final time / simulation time (s)
tvec_FFD    =       0:Ts_FFD:Tend_FFD;  % time vector (s)

% Initialize simulation output
N_FFD               =       length(tvec_FFD);   % number of samples
zout_FFD            =       zeros(10,N_FFD);     % matrix with states
uout_FFD            =       zeros(2,N_FFD);     % matrix with inputs
zout_FFD(:,1)       =       z0;                 % matrix of the states z(t+1) 
uout_FFD(:,1)       =       u;                  % matrix of the inputs
% theta_FFD           =       zeros(1,N_FFD);     % yaw angle
% theta_FFD(1,1)      =       theta;              % yaw angle initialization
% theta_dot           =       0;                  % yaw rate initialization

% Run simulation

% Computation of zout_FFD using Forward Euler Method
tic
for ind=2:N_FFD
    zdot                =      robot_dyn_model(0,zout_FFD(:,ind-1),uout_FFD(:,ind-1),0,th);          % derivatives of the state variables
    zout_FFD(:,ind)     =      zout_FFD(:,ind-1)+Ts_FFD*zdot;                                                  % update t given t-1                                   
%     theta_FFD(1,ind)    =      zout_FFD(3,ind);                                                                 % update of the yaw angle                                               
%     theta_dot           =      (theta_FFD(1,ind)-theta_FFD(1,ind-1))/Ts_FFD;
    %uout_FFD(:,ind)     =      uout_FFD(:,1);
%   if (ind >= round(N_FFD/2))
%      uout_FFD(:,ind) = [5 0]';  
%   end
end
t_FFD = toc

% Plot the results
figure(1),p1 = plot(zout_FFD(1,:),zout_FFD(2,:)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)'), title('Trajectory YX'), hold on
figure(2),p2 = plot(tvec_FFD,zout_FFD(3,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)'), title('Yaw vs time'), hold on


%% Simulation with ode45

% Time integration parameters
Ts_o45      =       0.1;               % sampling time (s)
Tend_o45    =       100;                % final time (s)
tvec_o45    =       0:Ts_o45:Tend_o45;  % time vector (s)

% Initialize simulation output
N_o45               =       length(tvec_o45);   % number of samples
zout_o45            =       zeros(7,N_o45);     % matrix with states
uout_o45            =       zeros(2,N_o45);     % matrix with inputs
Fout_o45            =       zeros(7,N_o45);     % matrix with forces
zout_o45(:,1)       =       z0;
uout_o45(:,1)       =       u;
theta_o45           =       zeros(1,N_o45);

% Run simulation
tic
theta_dot = 0;
for ind=2:N_o45
    zout_temp           =   ode45(@(t,z)robot_dyn_model(t,z,uout_o45(:,ind-1),0,th, theta_dot),[0 Ts_o45],zout_o45(:,ind-1));
    zout_o45(:,ind)     =   zout_temp.y(:,end);
    theta_o45(1,ind)    =   zout_o45(3,ind);   
    theta_dot           =   (theta_o45(1,ind)-theta_o45(1,ind-1))/Ts_FFD;
    %uout_o45(:,ind)     =   uout_o45(:,1);
end
t_o45 = toc



% Plot the results
figure(1),p3 = plot(zout_o45(1,:),zout_o45(2,:)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)'), title('Trajectory YX'),
figure(2),p4 = plot(tvec_o45,zout_o45(3,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)'), title('Yaw vs time')

h1 = [p1 p3]';
h2 = [p2 p4]';


legend(h1, 'FFD', 'o45');
legend(h2, 'FFD', 'o45');

