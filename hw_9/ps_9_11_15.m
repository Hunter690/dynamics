% Solve ODEs for Precessing, mutating, spinning gyro

clc, clear, close all

% Constants
g = 9.8; % m/s^2
m = .1; % kg
r = .2; % m
L = .2; % m

timeSpan = 0:.05:4;

% Given initial conditions:
theta_0 = 0;
dtheta_0 = 0;
phi_0 = 20*pi/180; % radians
dphi_0 = 0;
w_c = 300*2*pi/60; % rpm to rad/sec
initialConditions = [theta_0, dtheta_0, phi_0, dphi_0, w_c];

[time, solutions] = ode45(@(t, y) gyroODE(t, y, g, m, r, L), timeSpan, initialConditions);

% solutions columns:
%   1. theta
%   2. dtheta
%   3. phi
%   4. dphi
%   5. w_c

theta = solutions(:, 1) * 180/pi; % converted back to degrees for plot
figure(1);
plot(time, theta);
title("Precession: Theta (degrees) vs. Time (s)")
xlabel("Time (s)")
ylabel("Theta (degrees)")

phi = solutions(:, 3) * 180/pi;
figure(2);
plot(time, phi);
title("Nutation: Phi (degrees) vs. Time (s)")
xlabel("Time (s)")
ylabel("Phi (degrees)")

dtheta = solutions(:, 2) * 180/pi;
dphi = solutions(:, 4) * 180/pi;
w_c = solutions(:, 5);
ddtheta = -2*dphi.*(r^2*w_c+4*L^2*cos(phi).*dtheta)/(sin(phi)*(r^2+4*L^2));
ddphi =  sin(phi).*((4*L^2-r^2)*cos(phi).*dtheta.^2+2*r^2*w_c.*dtheta+4*g*L)/(r^2+4*L^2);
dw_c = -dphi.*(sin(phi).*dtheta+2*(r^2*w_c+4*L^2*cos(phi).*dtheta)/(tan(phi)*(r^2+4*L^2)));

% Plot H_bz as well using equation from 11.4 c
H_bz = 1/2*m*r^2*(dw_c+sin(phi).*dphi.*dtheta-cos(phi).*ddtheta);
figure(3);
plot(time, H_bz);
title("Angular Momentum vs. Time (s)")
xlabel("Time (s)")
ylabel("H (rad/s)")

% Plot K as well using equation from 11.4 b
K = 1/2*m*(1/2*r^2*w_c.^2+(1/4*r^2+L^2)*dphi.^2+(1/2*r^2+...
    (L^2-1/4*r^2)*(sin(phi)).^2).*dtheta.^2-r^2*w_c.*cos(phi).*dtheta);
U = m*g*L*cos(phi);
mechanicalEnergy = K + U;
figure(4);
plot(time, mechanicalEnergy);
title("Mechanical Energy vs. Time (s)")
xlabel("Time (s)")
ylabel("Mechanical Energy (J)")

% Differential Function
function dydt = gyroODE(t, y, g, m, r, L)
dydt = zeros(2, 1);

% first input theta ODE
% y(1) = theta, y(2) = dy(1) = dtheta, dydt(2) = ddy(1) = ddtheta
dydt(1) = y(2);
dydt(2) = -2*y(4)*(r^2*y(5)+4*L^2*cos(y(3))*y(2))/(sin(y(3))*(r^2+4*L^2));

% y(3) = phi, y(4) = dy(3) = dphi, dydt(4) = ddy(3) = ddphi
dydt(3) = y(4);
dydt(4) = sin(y(3))*((4*L^2-r^2)*cos(y(3))*y(2)^2+2*r^2*y(5)*y(2)+4*g*L)...
    /(r^2+4*L^2);

% y(5) = w_c, dy(5) = dw_c
dydt(5) = -y(4)*(sin(y(3))*y(2)+2*(r^2*y(5)+4*L^2*cos(y(3))*y(2))/...
    (tan(y(3))*(r^2+4*L^2)));;
end