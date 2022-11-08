clear all
close all
clc 

%% Import road reference

load('road.mat')

ref_road = data.RoadSpecifications.Centers;
x_ref = ref_road(:,1);
y_ref = ref_road(:,2);

ref = [x_ref, -y_ref];

size = length(ref);

%% Simulink simulation

sim('path_following_PID_v2.slx')

%% multiple xy graph
figure(10)
p_ref = plot(ref_data(:,1), ref_data(:,2), '--r*', 'linewidth', 1), grid on, ylabel('Y[m]'), xlabel('X[m]'), title('Actual vs reference trajectory'), hold on
p_fdb = plot(fdb_data(:,1), fdb_data(:,2), 'b', 'linewidth', 2), grid on

h = [p_ref p_fdb]'

legend(h, 'reference trajectory', 'actual trajectory')

%% Visualizer

X = fdb_data(:,1);
Y = fdb_data(:,2);
THETA = fdb_data(:,3);

FDB = [X Y THETA];
robot = struct;
robot.image = imread("robot.jpg");
robot.rotation = 0;
robot.centre = [500,500];
robot.length = 2;

figure(10)
h = plot_vehicle(FDB,'fps', 100, 'model', robot)
