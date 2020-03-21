% PS 6 9.7 part d

% Want to plot results for x vs. t 

% Given initial conditions:
m = 200; % kg
b = 300; % N*sec/meter
k = 139.2; % N/m
timeSpan = [0, 10];
x_0 = -40; % m
dx_0 = 0;

initialConditions = [x_0, dx_0];

[time, y] = ode45(@(t, y) rocketSlideODE(t, y, m, b, k), timeSpan, initialConditions);

figure(1);
% y is a 2 column double
% the first column is position, the second is velocity
plot(time, y(:,1));

title("Position vs. Time")
xlabel("Time (seconds)")
ylabel("x(t) in meters")

% Differential Function
function dydt = rocketSlideODE(t, y, m, b, k)
dydt = zeros(2, 1);

dydt(1) = y(2);
dydt(2) = -b/m*y(2)-k/m*y(1);
end