% PS 6 9.14

% Want to plot results for theta vs. t 

% Given initial conditions:
m = 100; % kg
g = 9.81; % m/s^2
L_0 = 50; % initial rope length in meters
s = 2; % retrieval speed in m/s
timeStep = 0:.02:24.92; % between 0 and 24.92 sec with .02 sec time step
theta_0 = pi/180; % 1 degree in radians
dtheta_0 = 0; % radians/sec

initialConditions = [theta_0, dtheta_0];

[time, theta] = ode45(@(t, y) helicopterBucketODE(t, y, m, g, L_0, s), timeStep, initialConditions);

figure(1);
% position is a 2 column double
% the first column is angle (radians), the second is angular velocity
plot(time, theta(:,1)*180/pi);

title("Theta (degrees) vs. Time")
xlabel("Time (seconds)")
ylabel("Theta (degrees)")

% Differential Function
function dydt = helicopterBucketODE(t, y, m, g, L_0, s)
dydt = zeros(2, 1);

dydt(1) = y(2);
dydt(2) = (4*y(2)-g*sin(y(1)))/(L_0-s*t);
end