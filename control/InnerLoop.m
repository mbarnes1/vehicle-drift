function u = InnerLoop(x,e_x,pars)
    % Mode 1 - Steering mode
    [delta, FxR, FyF] = Mode1(x,e_x,pars);
    
    % Mode 2 - Drive force mode
    if FyF >= pars.mu*pars.FzR
        [delta, FxR] = Mode2(err, pars);
    end
    u = [delta; FxR];
end

function [delta_des,FxR_des,FyF] = Mode1(x,e_x,pars)
    % Extract parameters and control constants
    a = pars.a;
    m = pars.m;
    mu = pars.mu;
    FxR_eq = pars.FxR_eq;
    r_eq = pars.r_eq;
    FzR = pars.FzR;
    K_beta = pars.K_beta;
    K_r = pars.K_r;
    K_Ux = pars.K_Ux;
    
    % Extract states and errors
    r = x(1);
    Ux = x(3);
    e_beta = e_x(1);
    e_r = e_x(2);
    e_Ux = e_x(3);
    
    % Compute constants
    [k1,k2] = Compute_ks(x(3), pars);

    % Compute desired rear drive force and resulting lateral force
    FxR_des = FxR_eq - m*K_Ux * e_Ux;
    FyR = NaN;  % TODO - find out what this is supposed to be.
                % FyR is the computed rear lateral force when operating in
                % mode 1. Note that in mode 1, the rear lateral force is
                % not treated as an input, but rather as an additional term
                % in the dynamic surface controller command that is a
                % function of the vehicle state and FxR_des
    
    % Compute desired front lateral force
    FyF = 1/k1 * ( k2 * FyR - K_beta^2 * e_beta - K_beta * r_eq - ...
        (K_beta + K_r) * e_r );
    
    % Compute desired steering angle
    delta_des = FyF2delta(FyF, x, pars);
end

function [delta_des,FxR_des] = Mode2(x,e_x,pars)
    % Extract parameters and control constants
    a = pars.a;
    m = pars.m;
    mu = pars.mu;
    FxR_eq = pars.FxR_eq;
    r_eq = pars.r_eq;
    FzF = pars.FzF;
    FzR = pars.FzR;
    K_beta = pars.K_beta;
    K_r = pars.K_r;
    K_Ux = pars.K_Ux;
    
    % Extract states and errors
    r = x(1);
    Ux = x(3);
    e_beta = e_x(1);
    e_r = e_x(2);
    e_Ux = e_x(3);
    
    % Compute desired front lateral force - saturated
    FyF_des = mu * FzF;
    
    % Compute desired steering angle
    delta_des = FyF2delta(FyF_des, x, pars);
    
    % Compute desired rear lateral force
    FyR_des = 1/k2 * (k1 * FzF + K_beta^2 * e_beta + K_beta * r_eq + ...
        (K_beta + K_r) * e_r );
    
    % Compute desired rear longitudinal force
    FxR_des = sqrt((mu*FzR)^2 - (FyR_des)^2);
end

function [k1,k2] = Compute_ks(Ux,pars)
    % Extract parameters
    a = pars.a;
    b = pars.b;
    Iz = pars.Iz;
    Kbeta = pars.K_beta;
    m = pars.m;
    
    k1 = a/Iz - Kbeta / (m * Ux);
    k2 = b/Iz + Kbeta / (m * Ux);
end

function delta = FyF2delta(FyF,x,pars)
    % Extract parameters
    a = pars.a;
    
    % Extract states
    beta = x(1);
    r = x(2);
    Ux = x(3);

    % Map front lateral force to a desired front tire slip angle and
    % compute the corresponding steer angle command
    alphaF = InverseFiala('front',FyF,pars); % TODO: implement inverse Fiala
    delta = alphaF - atan(beta + a/Ux*r);
end

