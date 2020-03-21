% Problem Set #5 Question 7.16 c)
% Asked to simulte the motion of a swinging spring for 0 <= t <= 16 seconds

% initialize function symbolic variables
syms g m Ln k t theta(t) y(t) Y

% Want to plot results for theta vs. t 

% set up differential equations
dy = diff(y);
d2y = diff(y, 2);
dtheta = diff(theta);
d2theta = diff(theta, 2);

% differential equation expressions
firstEquation = d2y == g*cos(theta)+(Ln+y)*dtheta^2-k*y/m;
secondEquation = d2theta == -(g*sin(theta)+2*dtheta*dy)/(Ln+y);

vectorField = odeToVectorField([firstEquation, secondEquation]);

specialFunction = matlabFunction(vectorField, 'vars', {'t', 'Y', 'g', 'm', 'Ln', 'k'});

% Given initial conditions:
%   y(0) = 0.2 m
%   theta(0) = 1 degree
%   dy(0)/dt = dtheta(0)/dt = 0
g = 9.8; % m/s^2
m = 1; % kg
Ln = 0.5; % m
k = 100; % N/m
timeSpan = [0, 16];
y_0 = 0.2;
dy_0 = 0;
theta_0 = 1*pi/180; % radians
dtheta_0 = 0;

initialConditions = [theta_0, dtheta_0, y_0, dy_0];

[time, solutions] = ode45(@(t, Y) specialFunction(t, Y, g, m, Ln, k), timeSpan, initialConditions);

% solutions consists of angle and y
theta = solutions(:, 1) * 180/pi;

figure(1);
plot(time, theta);

title("Result with Ln = 0.5m")
xlabel("Time (seconds)")
ylabel("Pendulum angle (degrees) when Ln = 0.5m")

% calculate result for new Ln
Ln = 0.3; % m
[time2, solutions2] = ode45(@(t, Y) specialFunction(t, Y, g, m, Ln, k), timeSpan, initialConditions);
theta2 = solutions2(:, 1) * 180/pi;

figure(2);
plot(time2, theta2);

title("Result with Ln = 0.3m")
xlabel("Time (seconds)")
ylabel("Pendulum angle (degrees) when Ln = 0.3m")