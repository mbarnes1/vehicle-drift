function [u_plus, mode] = Controller(x,u,pars)
%% Compute state error and extract sideslip error
e_x = x - pars.x_eq;
e_beta = e_x(1);

%% Sideslip Controller - compute desired yaw rate for sideslip control
r_des = SideslipController(e_beta, pars);

%% Use r_des instead of r_eq for our controller
e_r = x(2) - r_des;
e_x(2) = e_r;

%% Inner looop controller 
[u_plus, mode] = InnerLoop(x, e_x, pars);

%% Limit steering angle
u_plus(1) = min(pars.delta_max, u_plus(1));
u_plus(1) = max(-pars.delta_max, u_plus(1));

end

function r_des = SideslipController(e_beta, pars)
    r_des = pars.r_eq + pars.K_beta*e_beta;
end