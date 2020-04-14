% Solve ODEs for Precessing, mutating, spinning gyro

clc, clear, close all

syms g m r L t theta(t) phi(t) w_c(t) Y

% Set up differential equations
dtheta = diff(theta);
d2theta = diff(theta, 2);
dphi = diff(phi);
d2phi = diff(phi, 2);
dw_c = diff(w_c);

% differential equation expressions
firstEquation = d2theta == -2*dphi*(r^2*w_c+4*L^2*cos(phi)*dtheta)/(sin(phi)*(r^2+4*L^2));
secondEquation = d2phi == sin(phi)*((4*L^2-r^2)*cos(phi)*dtheta^2+2*r^2*w_c*dtheta+4*g*L)/(r^2+4*L^2);
thirdEquation = dw_c == -dphi*(sin(phi)*dtheta+2*(r^2*w_c+4*L^2*cos(phi)*dtheta)/(tan(phi)*(r^2+4*L^2)));

vectorField = odeToVectorField([firstEquation, secondEquation, thirdEquation]);
specialFunction = matlabFunction(vectorField, 'vars', {'t', 'Y', 'g', 'm', 'r', 'L'});

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
w_c = 300/60; % rotations per second
initialConditions = [theta_0, dtheta_0, phi_0, dphi_0, w_c];

[time, solutions] = ode45(@(t, Y) specialFunction(t, Y, g, m, r, L), timeSpan, initialConditions);

% % solutions consists of angle and y
% theta = solutions(:, 1) * 180/pi;
% figure(1);
% plot(time, theta);
% title("Precession: Theta (degrees) vs. Time (s)")
% xlabel("Time (s)")
% ylabel("Theta (degrees)")

% phi = solutions(:, 2) * 180/pi;
% figure(2);
% plot(time, phi);
% title("Nutation: Phi (degrees) vs. Time (s)")
% xlabel("Time (s)")
% ylabel("Phi (degrees)")