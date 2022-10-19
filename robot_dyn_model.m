function zdot=robot_dyn_model(t,z,u,d,th)
% 

%% Read parameters, states and inputs
% Parameters
Ic      =       th(1,1);     %
Iw      =       th(2,1);     % 
d       =       th(3,1);     % 
mw      =       th(4,1);     % 
mc      =       th(5,1);     % 
Im      =       th(6,1);     % 
R       =       th(7,1);     % 
L       =       th(8,1);     % 

mT = mc+2*mw;
I = Ic+mc*d^2+2*mw*L^2+2*Im;

% Additional paramaeters



% States
xa                  =       z(1,1);    % inertial X position (m)
ya                  =       z(2,1);    % inertial Y position (m)
theta               =       z(3,1);    % body orientation - yaw angle(rad)
phir                =       z(4,1);    % angular displacement right wheel (rad)
phil                =       z(5,1);    % angular displacement left wheel (rad)
%phir_dot            =       z(6,1);    % angular velocity right wheel (rad/s)    
%phil_dot            =       z(7,1);    % angular velocity left wheel (rad/s)

zd = zeros(5,1);

xa_dot              =       zd(1,1);    % inertial X velocity (m)
ya_dot              =       zd(2,1);    % inertial Y velocity (m)
theta_dot           =       zd(3,1);   % body orientation - yaw rate angle(rad)
phir_dot            =       zd(4,1);   % angular velocity right wheel (rad)
phil_dot            =       zd(5,1);   % angular velocity left wheel (rad)


%eta = [phir_dot; phil_dot];

cth = cos(theta);
sth = sin(theta);

% Inputs
taur      =       u(1,1);     % driving/braking torque right wheel (N*m)
taul      =       u(2,1);     % driving/braking torque left wheel (N*m)

%% Compute lateral and longitudinal tyre forces



% S = [R/(2*L)*(L*cth-d*sth) R/(2*L)*(L*cth+d*sth);
%      R/(2*L)*(L*sth+d*cth) R/(2*L)*(L*sth-d*cth);
%      R/(2*L)               -R/(2*L)             ;
%      1                      0                   ;
%      0                      1];



M = [mT             0       -mT*d*sth   0   0;
     0              mT      mT*d*cth    0   0;
     -mT*d*sth  mT*d*cth        I       0   0;
     0              0           0       Iw  0;
     0              0           0       0   Iw];



V = zeros(5);
V(1,2) = -mT*d*theta_dot*cth;
V(2,2) = -mT*d*theta_dot*sth;


B = zeros(5,2);
B(4,1) = 1;
B(5,2) = 1;




%% Control algorithm

%Steady-state matrices

% M_bar = [Iw+R^2/(4*L^2)*(mT*L^2+I) R^2/(4*L^2)*(mT*L^2-I); 
%         R^2/(4*L^2)*(mT*L^2-I) Iw+R^2/(4*L^2)*(mT*L^2+I)];
% 
% V_bar = [0 R^2/(2*L)*mc*d*theta_dot;
%         -R^2/(2*L)*mc*d*theta_dot 0];



% Model equations

% zdot = zeros(7,1);
% 
% 
% 
% 
% zdot = [S*eta; zeros(2,1)] + [zeros(5,2); eye(2)]*inv(M_bar)*(u-V_bar*eta);


zdot = zeros(10,1);

for i=1:5
    zdot(i,1) = z(i+5,1);
end

zdot_temp2 = inv(M)*(-V*zd+B*u);

for i=1:5
    zdot(i+5,1) = zdot_temp2(i,1);
end




