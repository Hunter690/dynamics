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
    
% I_C_Oc can be broken down as a I_S_Oc + I_D_Oc
% I_S_Oc = I_S_Os +- m_s * d_u * d_v where +- depends on if it's moment or
% product of inertia
% Similarly, I_D_Oc +- m_d * d_u * d_v

% From the definition of center of mass: m_s*rOs_Oc + m_d*rOd_Oc = 0
% Find rOd_Oc = 1/(m_s + m_d)*m_s*rOs_Od
rcd_d = ms/(ms + md)*rsd_d;
rOd_Oc = -rcd_d;

% Solving for rOs_Oc = -m_d/m_s*rOc_Od
rOs_Oc = md/ms*rcd_d;

% Because I_S_Oc is symmetric, only need to calculate 6/9 values in matrix
I_S_Oc = zeros(3,3);

% For the elements along the diagonal, I_S_Oc = I_S_Os + m_s * d^2
I_S_Oc(1) = Is(1) + ms*(rOs_Oc(2)^2 + rOs_Oc(3)^2);
I_S_Oc(5) = Is(5) + ms*(rOs_Oc(1)^2 + rOs_Oc(3)^2);
I_S_Oc(9) = Is(9) + ms*(rOs_Oc(1)^2 + rOs_Oc(2)^2);

% For off diagonal elements, I_S_Oc = I_S_Os - m_s * d_u * d_v
I_S_Oc(2) = Is(2) - ms*rOs_Oc(1)*rOs_Oc(2);
I_S_Oc(4) = I_S_Oc(2);
I_S_Oc(3) = Is(3) - ms*rOs_Oc(1)*rOs_Oc(3);
I_S_Oc(7) = I_S_Oc(3);
I_S_Oc(6) = Is(6) - ms*rOs_Oc(2)*rOs_Oc(3);
I_S_Oc(8) = I_S_Oc(6);


% Same process for I_D_Oc
I_D_Oc = zeros(3,3);
m_dDSquared = md * sum(rOd_Oc.^2);

% Calculate diagonal elements
I_D_Oc(1) = Id(1) + md*(rOd_Oc(2)^2 + rOd_Oc(3)^2);
I_D_Oc(5) = Id(5) + md*(rOd_Oc(1)^2 + rOd_Oc(3)^2);
I_D_Oc(9) = Id(9) + md*(rOd_Oc(1)^2 + rOd_Oc(2)^2);

% For off diagonal elements, I_D_Oc = I_D_Os - m_d * d_u * d_v
I_D_Oc(2) = Id(2) - md*rOd_Oc(1)*rOd_Oc(2);
I_D_Oc(4) = I_D_Oc(2);
I_D_Oc(3) = Id(3) - md*rOd_Oc(1)*rOd_Oc(3);
I_D_Oc(7) = I_D_Oc(3);
I_D_Oc(6) = Id(6) - md*rOd_Oc(2)*rOd_Oc(3);
I_D_Oc(8) = I_D_Oc(6);

% Add I_S_Oc and I_D_Oc to find Ic
Ic = I_S_Oc + I_D_Oc;
end