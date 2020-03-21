function [sRd, swd, ddt_swd] = calculate_alphaomega(t,nRs,nws,ddt_nws)
%CALCULATE_ALPHAOMEGA takes in info about the ship's orientation in
%the inertial frame, and returns corresponding information about the
%orientation of the dock in the ship frame.
%   input arguments:
%       -t = current time
%       -nRs = rotation matrix from S to N (3x3)
%       -nws = angular velocity of S in N, expressed in S (3x1)
%       -ddt_nws = angular acceleration of S in N, expressed in S (3x1)
%   output arguments:
%       -sRd = rotation matrix from D to S (3x3)
%       -swd = angular velocity of D in S, expressed in S (3x1)
%       -ddt_swd = angular acceleration of D in S, expressed in S (3x1)

sRd = eye(3);
swd = zeros(3,1);
ddt_swd = zeros(3,1);

% Step 1: find sRd
% know sRd = sRn * nRd = transpose(nRs) * nRd
% D reference frame = N reference frame rotated by theta(t) = cos(.3t)
theta = cos(.3*t);

nRd = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
sRd = nRs' * nRd;


% Step 2: find nws in S
% know swd_S = swn_S + nwd_S = -nws_S+nwd_S
% nwd_N = theta_dot*n_z = [0, 0, -.3sin(.3t)]' which produces 3x1
nwd_N = [0, 0, -.3*sin(.3*t)]';

% find nwd in S frame, so need to multiply by sRn = nRs'
nwd_S = nRs'*nwd_N;

% solve for swd_S
swd = -nws + nwd_S;


% Step 3: find ddt_swd in S expressed in S
% Take the derivative in the N frame of swd
% Break swd into -nws_S + nwd_S like in line 29
% Sddt_swd = Nddt(swd) + swn x swd = Nddt(-nws_S + nwd_S) + swn x swd
% know that Nddt(nwd_S) = sRn*Nddt(nwd_N) where nwd_N is broken down in
% line 30 to produce 3x1
Nddt_nwd_N = [0, 0, -.09*cos(.3*t)]';

% find derivative in S frame
Nddt_nwd_S = nRs' * Nddt_nwd_N;

% add together the derviative of the angular velocities
ddt_swd = -ddt_nws + Nddt_nwd_S + cross(-nws, swd);

end