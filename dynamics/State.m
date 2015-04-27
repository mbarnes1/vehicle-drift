function vehicle_state_new = State(vehicle_state, x, dt)
%STATE Computes the new state based on past state and current trajectory
%   Inputs:
%       vehicle_state - X, Y, and Theta positions of vehicle
%       x - beta, r, and Ux system state
%       dt - timestep
%
%   Outputs:
%       vehicle_state_new
X = vehicle_state(1);
Y = vehicle_state(2);
Theta = vehicle_state(3);

beta = x(1);
r = x(2);
Ux = x(3);

Theta_new = Theta + r*dt;
Uy = Ux*tan(beta);
X_new = X + dt*(Ux*cos(Theta) - Uy*sin(Theta));
Y_new = Y + dt*(Ux*sin(Theta) + Uy*cos(Theta));

vehicle_state_new = [X_new; Y_new; Theta_new];

end

