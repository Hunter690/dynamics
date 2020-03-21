function [rds_s,vds_s] = calculate_pv(rsn_n,rdn_n,vsn_n,vdn_n,nRs,w)
% CALCULATE_PV takes in the relevant position, orientation, and velocity
% for a time instant, and returns the position and velocity of the docking
% station as measured from the spaceship
%   input arguments:
%       -rsn_n = position of S from N, expressed in N (3x1 vector)
%       -rdn_n = position of D from N, expressed in N (3x1 vector)
%       -vsn_n = velocity of S from N, expressed in N (3x1 vector)
%       -vdn_n = velocity of D from N, expressed in N (3x1 vector)
%       -nRs = rotation matrix S --> N (3x3)
%       -w = angular velocity of N relative to S, expressed in S (3x1)
%   output arguments:
%       -rds_s = pos of D from S, expressed in S (3x1 vector)
%       -vds_s = vel of D from S, expressed in S (3x1 vector)

rds_s = zeros(3,1);
vds_s = zeros(3,1);

% calculate the position of D from S, expressed in N
rds_n = rdn_n - rsn_n;

% multiply the position vector by the transpose rotation matrix to express
% position vector in S
rds_s = nRs' * rds_n;

% calculate the vector of D from S in reference frame N, expressed in N
vds_n_n = vdn_n - vsn_n;

% multiply the velocity vector by the transpose rotation matrix to get
% velocity vector in reference frame N, expressed in S
vds_n_s = nRs' * vds_n_n;

% need to have velocity vector in reference frame S expressed in S, 
% so use golden rule
vds_s = vds_n_s + cross(w, rds_s);

end


