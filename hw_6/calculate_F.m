function Fs = calculate_F(nRs,rsn_n,nvs_n,nRd,rdn_n,nvd_n,nad_n,nwd_d,...
    ddt_nwd_d, das_d)
%CALCULATE_F takes in info about the ship's and the dock's respective
%movement and a desired acceleration of the ship relative to the dock. It
%returns the net thrust (i.e. force) required to obey the commanded
%relative acceleration. The mass of the spaceship is 300kg.
%   input arguments:
%       -nRs = rotation of S in N (3x3)
%       -rsn_n = position of Os from On, expressed in N (3x1)
%       -nvs_n = velocity of Os in N, expressed in N (3x1)
%
%       -nRd = rotation of D in N (3x3)
%       -rdn_n = position of Od from On, expressed in N (3x1)
%       -nvd_n = velocity of Od in N, expressed in N (3x1)
%       -nad_n = acceleration of Od in N, expressed in N (3x1)
%       -nwd_d = angular velocity of D in N, expressed in D (3x1)
%       -ddt_nwd_d = angular acceleration of D in N, expressed in D (3x1)
%       -das_d = desired acceleration of Os in D, expressed in D (3x1)
%   output arguments:
%       -Fs = force that thrusters should apply to spaceship (3x1)

Fs = zeros(3,1);

% F_Os_S = m*N_a_Os_S
% Need to find N_a_Os_S
% From expansion, N_a_Os_S = [N_a_Od + N_angular_acceleration_D x
% r_Os_Od + N_w_D x N_w_D x r_Os_Od + 2 * N_w_D x D_v_Os + D_a_Os] 
% all expressed in S

% first solve for N_a_Od expressed in S
% N_a_Od_S = S_R_N * N_a_Od_N
S_R_N = nRs';
N_a_Od_S = S_R_N * nad_n;

% next solve for tangential acceleration: N_angular_acceleration_D x r_Os_Od
% = S_R_D * N_angular_acceleration_D_D * S_R_N[r_Os_On - r_Od_On]_N
% need to get S_R_D
S_R_D = nRs'* nRd;

tangentialAcceleration = cross(S_R_D * ddt_nwd_d, S_R_N*(rsn_n - rdn_n));

% next solve for centripetal acceleration: N_w_D x N_w_D x r_Os_Od
% first calculate N_w_D x r_Os_Od
innerCentripetalCross = cross(S_R_D * nwd_d, S_R_N*(rsn_n - rdn_n));
centripetalAcceleration = cross(S_R_D * nwd_d, innerCentripetalCross);

% next solve for coriolis acceleration: 2 * N_w_D x D_v_Os
% D_v_Os = N_v_Od - N_w_D x r_Od_On + N_v_Os - N_w_D x r_Os_On
D_v_Os = S_R_N * nvd_n + cross(S_R_D * -nwd_d, S_R_N * rdn_n) + ...
    S_R_N * nvs_n + cross(S_R_D * -nwd_d, S_R_N * rsn_n);
coriolisAcceleration = 2 * cross(S_R_D * nwd_d, D_v_Os);

% next solve for D_a_Os_S
D_a_Os_S = S_R_D * das_d;

% add previous parts to form N_a_Os_S
N_a_Os_S = N_a_Od_S + tangentialAcceleration + centripetalAcceleration + ...
    + coriolisAcceleration + D_a_Os_S;

% multiply by mass to get Fs
Fs = 300 * N_a_Os_S;

end