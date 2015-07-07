% Outputs gravity force expressed in Body fixed frame
% g acceleration due to gravity (kg.m/s^2)

function [Gravity_Force] = Gravity_Forces(Atmosphere, Quadrotor, xi);

g = Atmosphere.g;

phi   = xi(7,1);
theta = xi(8,1);
psi   = xi(9,1);

mass = Quadrotor.Mass_prop.Mass;

Gravity_Force = mass*g*[-sin(theta) sin(phi)*cos(theta) cos(theta)*cos(phi)];
Gravity_Force = mass*g*[-sin(theta) sin(phi)*cos(theta) cos(theta)*cos(phi)];