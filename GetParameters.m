function pars = GetParameters()

pars.t0 = 0;            % seconds
pars.T = 10;            % seconds
pars.dt = 1e-4;         % seconds
pars.g = 9.8;           % m / s^2
pars.m = 1724;          % kg
pars.Iz = 1300;         % kg / m^2
pars.a = 1.35;          % m
pars.b = 1.15;          % m
pars.L = pars.a+pars.b; % m
pars.CaF = 120000;      % N / rad
pars.CaR = 175000;      % N / rad
pars.mu = 0.55;         % dimensionless

end
