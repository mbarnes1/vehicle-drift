function [u, mode] = InnerLoop(x,e_x,pars)
    % Mode 1 - Steering mode
    [delta, FxR, success] = Mode1(x,e_x,pars);
    
    % Mode 2 - Drive force mode
    if success
        fprintf('Mode 1\n');
        mode = 1;
    else
        [delta, FxR] = Mode2(x, e_x, pars);
        fprintf('Mode 2\n');
        mode = 2;
    end
    u = [delta; FxR];
end

function [delta_des,FxR_des,success] = Mode1(x,e_x,pars)
    % Extract parameters and control constants
    a = pars.a;
    b = pars.b;
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
    beta = x(1);
    r = x(2);
    Ux = x(3);
    e_beta = e_x(1);
    e_r = e_x(2);
    e_Ux = e_x(3);
    
    beta_eq = pars.beta_eq;
    
    % Compute constants
    [k1,k2] = Compute_ks(x(3), pars);

    % Compute desired rear drive force and resulting lateral force
    FxR_des = FxR_eq - m * K_Ux * e_Ux;
    FxR_des = min(FxR_des, FxR_max);
    FxR_des = max(FxR_des, 0);
    
    % Compute rear lateral force based on current state
    % FyR_des = a*m/L*r*Ux;   % Page 42 of Thesis - is this valid for normal use?
    % alphaR = atan(beta - b/Ux*r);
    % FyR_des = Fiala('rear', pars.CaR, pars.mu, pars.FzR, FxR_des, alphaR);
    % FyR_des should be calculated based on the saturation condition
    if beta_eq < 0
        FyR_des =  sqrt((mu*FzR)^2 - FxR_des^2);
    else
        FyR_des = - sqrt((mu*FzR)^2 - FxR_des^2);
    end
    
    % Compute desired front lateral force
    FyF_des = 1/k1 * ( k2 * FyR_des - K_beta^2 * e_beta - K_beta * r_eq - ...
        (K_beta + K_r) * e_r );
    
    % Compute desired steering angle if this is possible
    if abs(FyF_des) < pars.mu*pars.FzF
        % Compute desired steering angle
        delta_des = FyF2delta(x, FyF_des, pars);
        success = true;
    else
        delta_des = NaN;
        success = false;
    end
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
    
    beta_eq = pars.beta_eq;
    
    % Compute constants
    [k1,k2] = Compute_ks(x(3), pars);
    
    % Compute desired front lateral force - saturated
    % FyF_des = mu * FzF;
    if beta_eq < 0
        FyF_des =   mu * FzF;
    else
        FyF_des = - mu * FzF;
    end
    
    % Compute desired rear lateral force
    FyR_des = 1/k2 * (k1 * mu * FzF + K_beta^2 * e_beta + K_beta * r_eq + ...
        (K_beta + K_r) * e_r );
    FyR_des = min(FyR_des, mu*FzR);
    FyR_des = max(FyR_des, -mu*FzR);
    
    % Compute desired rear longitudinal force
    FxR_des = sqrt((mu*FzR)^2 - (FyR_des)^2);
    FxR_des = min(FxR_des, FxR_max);
    FxR_des = max(FxR_des, 0);
        
    if ~isreal(FxR_des)
        error('Mode 2 Error: FyR_des is larger than mu*FzR')
    end
    
    % Compute desired steering angle
    delta_des = FyF2delta(x, FyF_des, pars);
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

function delta = FyF2delta(x,FyF,pars)
    % Extract parameters
    a = pars.a;
    
    % Extract states
    beta = x(1);
    r = x(2);
    Ux = x(3);

    % Map front lateral force to a desired front tire slip angle and
    % compute the corresponding steer angle command
    alphaF = InverseFiala(FyF,pars);
    delta = - alphaF + atan(beta + a/Ux*r);
end

function alphaF = InverseFiala(FyF,pars)
    % Compute FyF_LUT for various alphas
    alphaF_LUT = (-30:0.001:30)*pi/180;
    FyF_LUT = Fiala('front', pars.CaF, pars.mu, pars.FzF, 0, alphaF_LUT);
    
    difference = abs(FyF - FyF_LUT);
    [~, ind] = min(difference);
    alphaF = alphaF_LUT(ind);
    
    % Compute forward Fiala model to verify precision
    FyF_test = Fiala('front', pars.CaF, pars.mu, pars.FzF, 0, alphaF);
    
%     fprintf('FyF: %.2f Test: %.2f Alpha: %.2f\n', FyF, FyF_test, alphaF);
    
    if (abs((FyF_test - FyF)/FyF) > 0.05) && (abs((FyF_test - FyF)) > 200)
%         fprintf('Inverse Fiala errors: %.2f, %.2f\n', ...
%             (FyF_test - FyF)/FyF, FyF_test - FyF);
%         error('Inverse Fiala model not accurate')
    end
    
end









