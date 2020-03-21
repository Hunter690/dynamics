function [rcd_d, Ic] = calculate_Ic(Id,Is,md,ms,rsd_d)
%CALCULATE_IC takes in info about the ship's and the dock's respective
%c.o.m. locations and computes a center of mass and combined inertia tensor
%(about the center of mass) for the dock-spaceship system.
%   input arguments:
%       -Id = inertia matrix for dock about Od, expressed in D (3x3)       
%       -Is = inertia matrix for spaceship about Os, expressed in D (3x3)
%       -md = mass of dock (Scalar)
%       -ms = mass of spaceship (Scalar)
%       -rsd_d = position of Os from Od, expressed in D (3x1)
%
%   output arguments:
%       -rcd_d = position of Oc from Od, expressed in D (3x1)
%       -Ic = combined inertia matrix for dock-spaceship system about Oc,
%             expressed in D (3x3) 

rcd_d = zeros(3,1);
Ic = zeros(3,3);
    
end