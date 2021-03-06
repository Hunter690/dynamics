function [x_dot, T] = calculate_pendulum_motion(t,x,m,l,g)
%CALCULATE_PENDULUM_MOTION takes time, state, and parameters for an N-link
%pendulum system and returns state derivative.
%   input arguments:
%       -x = state at time t: N angles and angular velocities (2Nx1)
%       -m = masses of the pendulums (Nx1)
%       -l = link lengths of the pendulums (Nx1)
%       -g = gravity (positive scalar)
%   output:
%       - x_dot = state derivative at time t (2Nx1)
%       - T = tension in each pendulum link at time t (Nx1)

numberOfParticles = numel(m);
x_dot = zeros(2*numberOfParticles,1);

% Need to create a systems of equations formatted as Az = b
% where A is a 2Nx2N matrix, z is a 2Nx1 vector that will hold the theta 
% double dots and the tensions 
% [ddtheta_1, ddtheta_2... ddtheta_N, T_1, T_2... T_N], and b is 2Nx1
A = zeros(2*numberOfParticles);
b = zeros(2*numberOfParticles,1);

% For each mass, need to calculate it's acceleration knowing that
% N_a_An = N_a_A(n-1) + N_dw_a_n x rAn_A(n-1) + N_w_an x (N_w_an x
% rAn_A(n-1))
% n orthonogonal basis are selected where each basis a^n has a_x has a
% direction along the rope to the next particle and a_z out of the plane
% All angles in given vector x are relative to the unit vector n_x which
% points vertically down, so N_w_an = dtheta_n and N_dw_an = ddtheta_n
% Note that n is always an integer and A_n represents the nth particle with
% mass m[n]

% Because of the way that angles are given, deciding to do all calculations
% in the newtonian reference frame

% For each particle from 1 to n-1, there are three forces:
%   1. gravity which is always m[n]*g in the n_x direction
%   2. T_left will always act in the -an_x direction
%       a. Note that this will need to be put in the N basis
%   3. T_right will always act in the a(n+1)_x direction
%       a. Note that this will need to be put in the N basis

% x = [theta_1, theta_2... theta_n, dtheta_1, dtheta_2...
% dtheta_n]'
% x_dot = [dtheta_1, dtheta_2... dtheta_n, ddtheta_1, ddtheta_2...
% ddtheta_n]'

dtheta = x(numberOfParticles+1: end);

% Set aside variables for previous particle's acceleration
previousParticleT_left = [0, 0, 0];
previousParticleT_right = [0, 0, 0];

% Set aside variable for current particle rotation matrix
theta1 = x(1);
N_R_ax = [cos(theta1), sin(theta1), 0];

% for loop through every particle
for n = 1:numberOfParticles
    % Calculate the position of the particle relative to the previous
    % particle using l(n)*an_x = l(n)*N_R_an*an_x
    rAn_A_nminus1 = l(n)*N_R_ax;
    
    % Calculate the acceleration of the particle using the previous particle's
    % acceleration + N_dw_a_n x rAn_A(n-1) + N_w_an x (N_w_an x rAn_A(n-1))
    % Note the need to calculate each term individually in order to put in
    % the matrix more easily
    % N_dw_a_n x rAn_A(n-1)
    ddthetaCoefficient = cross([0, 0, 1], rAn_A_nminus1);
    % N_w_an x (N_w_an x rAn_A(n-1))
    N_w_an = [0, 0, dtheta(n)];
    doubleOmega = cross(N_w_an, cross(N_w_an, rAn_A_nminus1));
    
    % For the previous particle's acceleration, replace it with the
    % net force of the previous particle divided by the previous particle's
    % mass
    % Calculate the coefficients of the tension
    m_n = m(n);
    T_n = 1/m_n*N_R_ax + previousParticleT_right;
    
    % Fill matrix A with ddtheta and tensions terms for n_x and n_y
    % directions
    row = 2*n-1;
    A(row, n) = ddthetaCoefficient(1);
    A(row, numberOfParticles + n - 1) = -previousParticleT_left(1);
    A(row, numberOfParticles + n) = T_n(1);

    A(row+1, n) = ddthetaCoefficient(2);
    A(row+1, numberOfParticles + n - 1) = -previousParticleT_left(2);
    A(row+1, numberOfParticles + n) = T_n(2);
        
    % Fill matrix b with terms that result from N_w_an x (N_w_an x rAn_A(n-1))
    b(row) = -doubleOmega(1);
    b(row+1) = -doubleOmega(2);
    
    % The very last particle does not have a tension force on the right side
    if n ~= numberOfParticles
        % Calculate the next particle rotation matrix N_R_an
        theta = x(n+1);
        N_R_ax_plus = [cos(theta), sin(theta), 0];
    
        T_n_right = 1/m_n*N_R_ax_plus;
        A(row, numberOfParticles + n + 1) = -T_n_right(1);
        A(row+1, numberOfParticles + n + 1) = -T_n_right(2);
        
        % Set this particle's tensions for the next particle
        previousParticleT_left = 1/m_n*N_R_ax;
        previousParticleT_right = T_n_right;

        % Already calculated next rotation matrix
        N_R_ax = N_R_ax_plus;
    end
end

% the first particle's gravitational force does not cancel
b(1) = b(1) + g*m(1);
    
% Calculate backslash
z = A\b;

% Combine dtheta with the first half of z (ddtheta) to form x_dot
x_dot(1:numberOfParticles) = dtheta;
x_dot(numberOfParticles + 1: end) = z(1:numberOfParticles);

% Set T to the last half of z
T = z(numberOfParticles + 1: end)';
end