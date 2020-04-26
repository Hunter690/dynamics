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

    N = numel(m);
    x_dot = zeros(2*N,1);
    
end