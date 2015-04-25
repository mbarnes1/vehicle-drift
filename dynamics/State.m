function [ X_new, Y_new, Theta_new ] = State( X, Y, Theta, beta, r, Ux, dt )
%STATE Computes the new state based on past state and current trajectory
%   Inputs:
%       X - Global X [m]
%       Y - Global Y [m]
%       Theta - Global orientation
%       beta - vehicle sideslip angle
%       r - yaw rate
%       Ux - longitudinal velocity
%       dt - time step
%
%   Outputs:
%       X_new
%       Y_new
%       Theta_new

Theta_new = Theta + r*dt;
Uy = Ux*tan(beta);
X_new = X + dt*(Ux*cos(Theta) - Uy*sin(Theta));
Y_new = Y + dt*(Ux*sin(Theta) + Uy*cos(Theta));

end

