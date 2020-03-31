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

% x is a vector that contains the positions and velocitys of N particles in
% the N reference frame. All the positions are listed first and then all
% the velocities follow: [r1_x, r1_y, r1_z... r2_x, r2_y, r2_z, v1_x, v1_y,
% v1_z... vn_x, vn_y, vn_z]
% Want to use force of gravity equation to solve for the acceleration of
% the N particles.
% The acceleration of the N particles are strictly caused by the
% gravitational forces of the particles on each other in this problem.
% For each particle, the force the particle feels from each other particle
% = -G*m_current_particle*m_other_particle/distance_between_particles_cubed
% *position_vector_from_other_particle_to_current_particle
numberOfParticles = length(m);

% For each particle, need to calculate the forces from the other particles
% Once the force on particle A by B is calculated, don't want to
% recalculate the force on particle B by A. We know the magnitude is the
% same, just the direction is the opposite, so we can form an NxN matrix
% where the the i,j entry is the force on particle i by j.
for particleI = 1:numberOfParticles
    % Create a force matrix, need 3x the rows because each particle force has
    % three components
    forceMatrix = zeros(3, numberOfParticles);

    % Get the mass of particle I for later force calculation
    IMass = m(particleI);
    
    % Get the position of particle I; multiply by 3 because each particle
    % has 3 indices in x to describe all components in position
    IPositionStartingIndex = (particleI - 1) * 3 + 1;
    r_I_On = x(IPositionStartingIndex:IPositionStartingIndex + 2);
    
    for particleJ = 1:numberOfParticles
        if particleI == particleJ
            % the gravitational force of a particle on itself is always 0
            continue
        end

        % Get the position of particle J
        JPositionStartingIndex = (particleJ - 1) * 3 + 1;
        r_J_On = x(JPositionStartingIndex:JPositionStartingIndex + 2);
        
        % Calculate the position vector from J to I
        % r_I_J = r_I_On + r_On_J
        r_I_J = r_I_On - r_J_On;
        
        % Calculate the distance between I and J
        particleSeparation = sqrt(sum(r_I_J.^2));
        
        % Calculate the force of from J on I
        F_I_J = -G*IMass*m(particleJ)/particleSeparation^3*r_I_J;
        
        % Put the force in the force matrix using 3 rows, and the j'th
        % column index 
        forceMatrix(:, particleJ) = F_I_J;
    end
    
    % Calculate the sum of row i to find the net force on particle I (note
    % that the net force should return a 3x1 vector)
    netForceOnI = [sum(forceMatrix(1, :)), sum(forceMatrix(2, :)), ...
        sum(forceMatrix(3, :))];
    
    % Take the force that was calculated and divide by the mass of particle I to
    % get the 3x1 acceleration vector of particle I
    N_a_I = netForceOnI/IMass;
    
    % Get the velocity of particle I from x; the velocity of particles are
    % in the second half of x after index = numberOfParticles*3. Then need
    % to add IPositionStartingIndex to get the velocity index
    IVelocityStartingIndex = numberOfParticles*3 + IPositionStartingIndex;
    N_v_I = x(IVelocityStartingIndex:IVelocityStartingIndex + 2);

    % Place the velocity and acceleration vectors in x_dot
    % The velocity should be placed where the old position was placed and
    % acceleration should be where the old velocity was
    x_dot(IPositionStartingIndex:IPositionStartingIndex + 2) = N_v_I;
    x_dot(IVelocityStartingIndex:IVelocityStartingIndex + 2) = N_a_I';
end

end
