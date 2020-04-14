% PS 9 11.7 part

clc, clear, close all

% Want to solve ODE for problem 6.18 and plot 
% w_x, w_y, w_z, H_x, H_y, H_z, H_magnitude, and K

% Given initial conditions:
w_x_0 = 7; % rad/sec
w_y_0 = .2; % rad/sec
w_z_0 = .2; % rad/sec
I_xx = 1; % kg/m^2
I_yy = 2; % kg/m^2
I_zz = 3; % kg/m^2

% Run Simulink Model
simulinkModel = 'angular_velocity.slx';
load_system(simulinkModel);
output = sim(simulinkModel,'SaveOutput','on');
w_x = output.w_x;
w_y = output.w_y;
w_z = output.w_z;

H_x = I_xx*w_x;
H_y = I_yy*w_y;
H_z = I_xx*w_z;

H_magnitude = sqrt(H_x.data.^2 + H_y.data.^2 + H_z.data.^2);

K = 1/2*(I_xx*w_x.data.^2 + I_zz*w_z.data.^2 + I_zz*w_z.data.^2);

% angular_velocity
figure(1);
plot(w_x);
hold on
plot(w_y);
plot(w_z);
hold off

title("Angular Velocity (rad/s)")
xlabel("time (s)")
ylabel("w (rad/s)")
legend("w_x", "w_y", "w_z")

% angular_momentum
figure(2);
plot(H_x);
hold on
plot(H_y);
plot(H_z);
plot(H_x.time, H_magnitude);
hold off

title("Angular Momentum (rad/s)")
xlabel("time (s)")
ylabel("H (rad/s)")
legend("H_x", "H_y", "H_z", "|H|")

% kinetic energy
figure(3);
plot(w_x.time, K);

title("Kinetic Energy (J)")
xlabel("time (s)")
ylabel("K (J)")