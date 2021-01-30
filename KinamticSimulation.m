%%Kinematic Simulation of Mobile Robot
clear all; clc; close all;

%Simulation Parameters
dt = 0.1;  %Step-Size
ts = 10;   %Simulation time
t = 0:dt:ts %TIme Span

%% Initial Conditions

x0 = 0;
y0 = 0;
psi0 = pi/4;

eta0 = [x0; y0; psi0];

eta(:,1) = eta0;

%% Loop started from here

for i = 1:length(t)
    psi = eta(3,i); %Current Orientation
    %Jacobian Matrix
    j_psi = [cos(psi), -sin(psi), 0;
             sin(psi), cos(psi), 0;
             0,0,1];
    u=0.1; %X-axis velocity in body fixed frame 
    v=0.05; %Y-axis Velocity in body fixed frame
    r=0; %Angular velocity in bodu fixed frame
    zeta(:,i) = [u;v;r];     
    
    eta_dot(:,i) = j_psi * zeta(:,i);
    
    eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); %Euler Method
    
end

%% Plotting Functions Starts from here
figure
plot(t, eta(1,1:i),'-r');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('x,[m]');

figure
plot(t, eta(2,1:i),'-g');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('y,[m]');

figure
plot(t, eta(3,1:i),'-b');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('/psi,[rad]');


    
    