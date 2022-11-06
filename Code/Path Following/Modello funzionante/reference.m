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