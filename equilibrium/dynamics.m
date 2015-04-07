function D = dynamics(Ux, delta, r, FxR, Beta)

% Define parameters
g = 9.8;        % m / s^2
m = 1724;       % kg
Iz = 1300;      % kg / m^2
a = 1.35;       % m
b = 1.15;       % m
L = a+b;        % m
CaF = 120000;   % N / rad
CaR = 175000;   % N / rad
mu = 0.55;      % dimensionless

% Compute lateral forces
alphaF = atan(Beta + a/Ux*r) - delta;
FzF = b*m*g/L;
FyF = fiala('front', CaF, mu, FzF, FxR, alphaF);

alphaR = atan(Beta - b/Ux*r);
FzR = a*m*g/L;
FyR = fiala('rear', CaR, mu, FzR, FxR, alphaR);

% Compute dynamics
D = zeros(3,1);
D(1) = 1/(m*Ux)*(FyF + FyR) - r;
D(2) = 1/Iz*(a*FyF-b*FyR);
D(3) = 1/m*(FxR-FyF*sin(delta))+r*Ux*Beta;

end