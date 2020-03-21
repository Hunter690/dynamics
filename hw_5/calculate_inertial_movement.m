function [nws, ddt_nws] = calculate_inertial_movement(P, V, A)
%CALCULATE_INERTIAL_MOVEMENT takes in velocity and acceleration readings
%V,A from sensors at the 4 ship locations stored P.
%   input arguments:
%       -P = position of sensors w.r.t. O_s, expressed in S (3x4)
%       -V = velocity of sensors w.r.t. O_n, expressed in S (3x4)
%       -A = acceleration of sensors w.r.t. O_n, expressed in S (3x4)
%
%   output arguments:
%       -nws = angular velocity of S in N, expressed in S (3x1)
%       -ddt_nws = angular acceleration of S in N, expressed in S (3x1)

nws = zeros(3,1);
ddt_nws = zeros(3,1);

% First solve for nws
% Subtracting velocities of each point with respect to O_n leads to:
% N_v_P(i+1) = -(r_P(i+1)/Os-r_P(i)/Os) x N_w_S
%     b      =          - Ti            x N_w_S
% Ti = -skew(P(:, i+1))
% bi = V(:, i+1)
% combine the Ti to form a single T so that we can solve for N_w_S using
% backslash:
% Note: because of MATLAB's 1 start index, P(:,1) is r_P0/Os
% find the skew symmetric matrices for all Pi position differences

P21 = P(:, 2);
P32 = P(:, 3);
P43 = P(:, 4);

T1 = skew(P21);
T2 = skew(P32);
T3 = skew(P43);
T = -[T1; T2; T3];

b1 = V(:, 2);
b2 = V(:, 3);
b3 = V(:, 4);
b = [b1; b2; b3];

nws = T\b;

% Next, solve for S_ddt_nws
% Subtracting accelerations of each point with respect to O_n leads to:
% N_a_P(i+1) - N_a_P(1) - N_w_S x N_w_S x (r_P(i+1)/Os-r_P(i)/Os)=
% -(r_P(i+1)/Os-r_P(i)/Os) x S_ddt_N_w_S
% Notice that Ti is the same as the once used to calculate N_w_S
% dbi = A(:, i+1) - A(:, 1) - N_w_S x N_w_S * skewP(i+1)(i)
db1 = A(:, 2) - A(:, 1) - cross(nws, cross(nws, P21));
db2 = A(:, 3) - A(:, 1) - cross(nws, cross(nws, P32));
db3 = A(:, 4) - A(:, 1) - cross(nws, cross(nws, P43));

db = [db1; db2; db3];

ddt_nws = T\db;

end

