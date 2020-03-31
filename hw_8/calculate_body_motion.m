function x_dot = calculate_body_motion(t,x,m,G)
% CALCULATE_BODY_MOTION takes time, state, and parameters for an N-body
% system and returns state derivative.
%   input arguments:
%       -x = state at time t: N 3D positions and velocities (6Nx1)
%       -m = masses of the bodies (Nx1)
%       -G = Newtonian constant of gravitation (scalar)
%   output arguments:
%       -x_dot = state derivative at time t: N velocities and accels (6Nx1)


    x_dot = zeros(size(x));
end
