function u = InnerLoop(err, x, pars)

% Get tire forces (synthetic control inputs)
[FyR, FyF] = YawController(err, pars);

% Compute direct control inputs from tire forces
u = TireForces2u(FyF, FyR, x, pars.a);

end

function [FyR, FyF] = YawController(err, pars)

if FyF < mu*FzR % Steering Mode
    mode = 'S';
    [FyR, FyF] = YawController_S(err, pars);
else % Drive Force Mode
    mode = 'D';
    [FyR, FyF] = YawController_D(err, pars);
end

end

function u = TireForces2u(FyF, FyR, x, a)
    Ux = x(3);
    r = x(2);
    beta = x(1);
    
    alphaF = NaN; % TODO: Invert brush tire model
    
    delta = alphaF  - atan(beta + a/Ux * r);
    FxR = sqrt( (mu * FzR)^2 - FyR^2 );
    u = [delta, FxR]';
end

function [FyR, FyF] = YawController_S(err, pars)
    [k1,k2] = Compute_ks(Ux, pars);
    r_eq = pars.r_eq;

    Kbeta = pars.Kbeta;
    Kr = pars.Kr;

    er = err(2);
    e_Ux = err(3);
    
    FxR = FxR_eq - m*K_Ux * e_Ux;
    FyR = NaN; % TODO: Figure out what this is supposed to be.
    FyF = 1/k1 * ( k2 * FyR - Kbeta^2 * eb - Kbeta * r_eq - (Kbeta + Kr) * er );
end

function [FyR, FyF] = YawController_D(err, pars)
    [k1,k2] = Compute_ks(Ux, pars);
    r_eq = pars.r_eq;
    Kbeta = pars.Kbeta;
    Kr = pars.Kr;

    FzF = NaN; % TODO: Figure out what this is supposed to be.
    ebeta = err(1);
    er = err(2);

    FyF = mu * FzF;
    FyR = 1/k2 * (k1 * FyF + Kbeta^2 * ebeta + Kbeta * r_eq + (Kbeta + Kr) * er );
end

function [k1,k2] = Compute_ks(Ux, pars)
    a = pars.a;
    Iz = pars.Iz;
    Kbeta = pars.Kbeta;
    m = pars.m;
    
    k1 = a/Iz - Kbeta / (m * Ux);
    k2 = b/Iz + Kbeta / (m * Ux);
end

