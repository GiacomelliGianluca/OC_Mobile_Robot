function zdot=robot_dyn_model(t,z,u,d,th)

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


% States
xa                  =       z(1,1);    % inertial X position (m)
ya                  =       z(2,1);    % inertial Y position (m)
theta               =       z(3,1);    % body orientation - yaw angle(rad)
phir                =       z(4,1);    % angular displacement right wheel (rad)
phil                =       z(5,1);    % angular displacement left wheel (rad)
phir_dot            =       z(6,1);    % angular velocity right wheel (rad/s)    
phil_dot            =       z(7,1);    % angular velocity left wheel (rad/s)



% Additional paramaeters
cth = cos(theta);
sth = sin(theta);

%theta_dot = R/(2*L)*phir_dot-(R/2*L)*phil_dot;
lam = [-sth     cth     0;
        cth     sth     L;
        cth     sth     -L];

b = inv(lam)*[0; R*phir_dot; R*phil_dot];

xa_dot      =    b(1,1);
ya_dot      =    b(2,1);
theta_dot   =    b(3,1);




% Inputs
taur      =       u(1,1);     % driving/braking torque right wheel (N*m)
taul      =       u(2,1);     % driving/braking torque left wheel (N*m)


%% Matrices


S = [R/(2*L)*(L*cth-d*sth) R/(2*L)*(L*cth+d*sth);
     R/(2*L)*(L*sth+d*cth) R/(2*L)*(L*sth-d*cth);
     R/(2*L)               -R/(2*L)             ;
     1                      0                   ;
     0                      1];




M_bar = [Iw+R^2/(4*L^2)*(mT*L^2+I)          R^2/(4*L^2)*(mT*L^2-I); 
        R^2/(4*L^2)*(mT*L^2-I)              Iw+R^2/(4*L^2)*(mT*L^2+I)];

V_bar = [0                              R^2/(2*L)*mc*d*theta_dot;
        -R^2/(2*L)*mc*d*theta_dot               0];



% Model equations

zdot = zeros(7,1);

zdot = [S*z(6:7,1); zeros(2,1)] + [zeros(5,2); eye(2)]*inv(M_bar)*(u-V_bar*z(6:7,1));







