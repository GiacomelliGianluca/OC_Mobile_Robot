function zdot=robot_dyn_model(t,z,u,d,th)

%% Read parameters, states and inputs
% Parameters
Im      =       th(1,1);     %
Iw      =       th(2,1);     % 
d       =       th(3,1);     % 
mw      =       th(4,1);     % 
mr      =       th(5,1);     % 
R       =       th(5,1);     % 
L       =       th(5,1);     % 

mT = mr+2*mw;
I = Im+mr*d^2+2*mw*L^2+2*Im;


% States
phi1      =       z(1,1);    % right wheel angular displacement (rad)
phi2      =       z(2,1);    % left wheel angular displacement (rad)


% Inputs
taur      =       u(1,1);     % driving/braking torque right wheel (N*m)
taul      =       u(2,1);     % driving/braking torque left wheel (N*m)

%% Inertia matrix


H = [Iw+R^2/(4*L^2)*(mT*L^2+I)      R^2/(4*L^2)*(mT*L^2-I);
    R^2/(4*L^2)*(mT*L^2-I)          Iw+R^2/(4*L^2)*(mT*L^2+I)];

%% Coriolis terms

K = [        0                     R^2/(2*L)*mr*d*theta_dot;
    R^2/(2*L)*mr*d*theta_dot            0];

%% States
q = [phi1;
    phi2];

%% Model equations


syms y(t)
zdot = odeToVectorField(H*diff(y, 2) == (-K*diff(y)) + tau);







