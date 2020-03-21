% PS 6 9.10 part g

% Want to plot results for y_B_0 vs. x_B_0 and y vs. x

% Given initial conditions:
g = 9.81; % m/s^2
d = .1; % meter
t = 0:.01:1.21;
v_x0 = 1; % m/s
v_y0 = 6; % m/s

x = v_x0 * t;
y = v_y0 * t - g*t.^2/2;
x_B_0 = x - d*cos(24*t);
y_B_0 = y - d*sin(24*t);

figure(1);
plot(x, y);
hold on
plot(x_B_0, y_B_0);
hold off

title("Projectile motion of a rigid body")
xlabel("x (meters)")
ylabel("y (meters)")
legend("y(t) vs. x(t)", "y_B_0 vs. x_B_0")