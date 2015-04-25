function u = InnerLoop(x,e_x,pars)
    % Mode 1 - Steering mode
    [delta, FxR, FyF] = Mode1(x,e_x,pars);
    
    % Mode 2 - Drive force mode
    if FyF >= pars.mu*pars.FzF
        [delta, FxR] = Mode2(x, e_x, pars);
        fprintf('Mode 2\n');
    else
        fprintf('Mode 1\n');
    end
    u = [delta; FxR];
end

function [delta_des,FxR_des,FyF_des] = Mode1(x,e_x,pars)
    % Extract parameters and control constants
    a = pars.a;
    m = pars.m;
    mu = pars.mu;
    L = pars.L;
    FzR = pars.FzR;
    FxR_eq = pars.FxR_eq;
    FxR_max = pars.FxR_max;
    r_eq = pars.r_eq;
    K_beta = pars.K_beta;
    K_r = pars.K_r;
    K_Ux = pars.K_Ux;
    
    % Extract states and errors
    r = x(2);
    Ux = x(3);
    e_beta = e_x(1);
    e_r = e_x(2);
    e_Ux = e_x(3);
    
    % Compute constants
    [k1,k2] = Compute_ks(x(3), pars);

    % Compute desired rear drive force and resulting lateral force
    FxR_des = FxR_eq - m*K_Ux * e_Ux;
    FxR_des = min(FxR_des, FxR_max);
    FxR_des = max(FxR_des, -FxR_max);
    
    % Compute rear lateral force based on current state
    FyR_des = a*m/L*r*Ux;   % Page 42 of Thesis - is this valid for normal use?
                        % If not, then we have to take the state derivative
                        % into account, which means taking the time-step
                        % into account or something... right?
                
    % Compute desired front lateral force
    FyF_des = 1/k1 * ( k2 * FyR_des - K_beta^2 * e_beta - K_beta * r_eq - ...
        (K_beta + K_r) * e_r );
    
    % Compute desired steering angle
    delta_des = FyF2delta(x, FyF_des, FxR_des, pars);
end

function [delta_des,FxR_des] = Mode2(x,e_x,pars)
    % Extract parameters and control constants
    mu = pars.mu;
    r_eq = pars.r_eq;
    FzF = pars.FzF;
    FzR = pars.FzR;
    FxR_max = pars.FxR_max;
    K_beta = pars.K_beta;
    K_r = pars.K_r;
    
    % Extract states and errors
    e_beta = e_x(1);
    e_r = e_x(2);
    
    % Compute constants
    [k1,k2] = Compute_ks(x(3), pars);
    
    % Compute desired front lateral force - saturated
    FyF_des = mu * FzF;
    
    % Compute desired rear lateral force
    FyR_des = 1/k2 * (k1 * mu * FzF + K_beta^2 * e_beta + K_beta * r_eq + ...
        (K_beta + K_r) * e_r );
    FyR_des = min(FyR_des, mu*FzR);
    FyR_des = max(FyR_des, -mu*FzR);
    
    % Compute desired rear longitudinal force
    FxR_des = sqrt((mu*FzR)^2 - (FyR_des)^2);
    FxR_des = min(FxR_des, FxR_max);
    FxR_des = max(FxR_des, -FxR_max);
        
    if ~isreal(FxR_des)
        error('Mode 2 Error: FyR_des is larger than mu*FzR')
    end
    
    % Compute desired steering angle
    delta_des = FyF2delta(x, FyF_des, FxR_des, pars);

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

function delta = FyF2delta(x,FyF,FxR,pars)
    % Extract parameters
    a = pars.a;
    
    % Extract states
    beta = x(1);
    r = x(2);
    Ux = x(3);

    % Map front lateral force to a desired front tire slip angle and
    % compute the corresponding steer angle command
    alphaF = InverseFiala(FyF,FxR,pars); 
    delta = - alphaF + atan(beta + a/Ux*r); % SIGN?!?!?!?!
end

function alphaF = InverseFiala(FyF,FxR,pars)
    % Compute FyF_LUT for various alphas
    alphaF_LUT = (-20:0.005:20)*pi/180;
    FyF_LUT = Fiala('rear', pars.CaR, pars.mu, pars.FzR, FxR, alphaF_LUT);
    
    difference = abs(FyF - FyF_LUT);
    [~, ind] = min(difference);
    alphaF = alphaF_LUT(ind);
    
    % Compute forward Fiala model to verify precision
    FyF_test = Fiala('rear', pars.CaR, pars.mu, pars.FzR, FxR, alphaF);
    
%     if (abs((FyF_test - FyF)/FyF) > 0.05) && (abs((FyF_test - FyF)) > 400)
%         error('Inverse Fiala model not accurate')
%     end
    
end









