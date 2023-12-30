classdef dynamics
    % Contains functions that represent translational motion of a spherical
    % object with proper lift, drag and body forces.

    methods (Static)

        function states = sphere_model(~,initial,ball_radius,ball_mass,air_density,drag_coefficient,gravity,angular_velocity)

            % velocities in x,y,z directions
            xdot = initial(4);
            ydot = initial(5);
            zdot = initial(6);
            velocity_magnitude = sqrt(xdot^2+ydot^2+zdot^2);

            % grouped constants
            A = (4/3) * (4 * pi^2 * ball_radius^3 * air_density);
            B = (1/2) * air_density * (pi * ball_radius^2) * drag_coefficient;
            C = ball_mass * gravity;

            % acceleration in x,y,z directions
            xddot = (A * (angular_velocity(2) * zdot - angular_velocity(3) * ydot) - B * velocity_magnitude * xdot) / ball_mass;
            yddot = (A * (angular_velocity(3) * xdot - angular_velocity(1) * zdot) - B * velocity_magnitude * ydot) / ball_mass;
            zddot = (A * (angular_velocity(1) * ydot - angular_velocity(2) * xdot) - B * velocity_magnitude * zdot - C) / ball_mass;

            states = [xdot;ydot;zdot;xddot;yddot;zddot];
        end   

        function [check, isterminal, direction] = reachesGround(~,r)
            check = r(3);       % value we want to be zero
            isterminal = 1;     % stop or not (1=stop,0=continue)
            direction = 0;      % either direction (-1=negative direction,1=positive direction,0=either)
        end

    end
end