% PS 6 9.16 part e

% Want to plot results for x vs. t 

% Given initial conditions:
m = 1; % kg
g = 9.81; % m/s^2
k = 400; % N/m
L_n = 4; % m
b = 4; % N*sec/meter
u = .4; % coefficient of friction
e_v = 1e-5; % avoids dividing by zero, must be much smaller than velocity
timeSpan = [0, 12];
x_0 = 4; % m
dx_0 = 0;

initialConditions = [x_0, dx_0];

[time, y] = ode45(@(t, y) springMassDamperODE(t, y, m, g, k, L_n, b, u, e_v), timeSpan, initialConditions);

figure(1);
% y is a 2 column double
% the first column is position, the second is velocity
plot(time, y(:,1));

title("Position vs. Time")
xlabel("time (seconds)")
ylabel("x (meters)")

% Differential Function
function dydt = springMassDamperODE(t, y, m, g, k, L_n, b, u, e_v)
dydt = zeros(2, 1);

dydt(1) = y(2);
dydt(2) = (20*cos(t)-b*y(2)-k*(y(1)-L_n)-u*m*g*y(2)/(abs(y(2))+e_v))/m;
end