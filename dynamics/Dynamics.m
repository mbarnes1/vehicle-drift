function D = Dynamics(state, control, pars)
% Extract state and control inputs
Beta    = state(1);
r       = state(2);
Ux      = state(3);
delta   = control(1);
FxR     = control(2);

% Define parameters
g = pars.g;
m = pars.m;
Iz = pars.Iz;
a = pars.a;
b = pars.b;
L = pars.L;
CaF = pars.CaF;
CaR = pars.CaR;
mu = pars.mu; 
FzF = pars.FzF;
FzR = pars.FzR;

% Compute lateral forces
alphaF = atan(Beta + a/Ux*r) - delta;
[FyF, ~] = Fiala('front', CaF, mu, FzF, FxR, alphaF);

alphaR = atan(Beta - b/Ux*r);
[FyR, sat_r] = Fiala('rear', CaR, mu, FzR, FxR, alphaR);

% Compute dynamics
D = zeros(4,1);
D(1) = 1/(m*Ux)*(FyF + FyR) - r;            % Beta dot
D(2) = 1/Iz*(a*FyF-b*FyR);                  % r dot
D(3) = 1/m*(FxR-FyF*sin(delta))+r*Ux*Beta;  % Ux dot
D(4) = sat_r;  % whether rear tire is saturated

if ~isreal(D)
    error('Complex dynamics')
end

end