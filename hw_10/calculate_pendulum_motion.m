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
% where z is a 2Nx1 vector that will hold the theta double dots 
% and the tensions [ddtheta_1, ddtheta_2... ddtheta_N, T_1, T_2... T_N]

% For each mass, need to calculate it's acceleration knowing that
% N_a_A_n = N_a_A_previous_n + N_dw_a_n x rAn_Ao + N_w_an x (N_w_an x
% rAn_Ao)
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

% x_dot = [dtheta_1, dtheta_2... dtheta_n, ddtheta_1, ddtheta_2...
% ddtheta_n]'

% for loop through every particle except for the last one

% Calculate the rotation matrix N_R_an
% Calculate the position of the particle using the previous particle's
% position + l[n]*an_x = rA(n-1)/Ao + l[n]*N_R_an*an_x

% Calculate the acceleration of the particle using the previous particle's
% acceleration + N_dw_a_n x rAn_Ao + N_w_an x (N_w_an x rAn_Ao)
% Note that for the previous particle's acceleration, replace it with the
% net force of the previous particle divided by the previous particle's
% mass

% Fill matrix A with all terms with tension and ddtheta

% Fill matrix b with terms that result from N_w_an x (N_w_an x rAn_Ao)


end