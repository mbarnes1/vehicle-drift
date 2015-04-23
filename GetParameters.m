function pars = GetParameters()

%% Timing
pars.t0 = 0;            % seconds
pars.T = 10;            % seconds
pars.dt = 1e-4;         % seconds

%% Initial state and control
pars.x0 = zeros(3,1);
pars.u0 = zeros(3,1);

%% Vehicle parameters
pars.g = 9.8;           % m / s^2
pars.m = 1724;          % kg
pars.Iz = 1300;         % kg / m^2
pars.a = 1.35;          % m
pars.b = 1.15;          % m
pars.L = pars.a+pars.b; % m
pars.CaF = 120000;      % N / rad
pars.CaR = 175000;      % N / rad
pars.mu = 0.55;         % dimensionless

% Compute the normal forces since those are assumed constant
pars.FzF = pars.b*pars.m*pars.g/pars.L;     % N
pars.FzR = pars.a*pars.m*pars.g/pars.L;     % N

%% Equilibrium point - hard coded
pars.delta_eq   = -12*pi/180;   % degrees
pars.Ux_eq      = 8;            % m / s

pars.r_eq       = 0.600;        % rad /s
pars.beta_eq    = -20.44*pi/180;% rad /s
pars.FxR_eq     = 2293;         % N
pars.FyF_eq     = 3807;         % N
pars.FyR_eq     = 4469;         % N

pars.x_eq = [pars.beta_eq; pars.r_eq; pars.Ux_eq];

%% Controller parameters
pars.K_beta = 2;
pars.K_r    = 4;
pars.K_Ux   = 0.846;

%% Inverse fiala look up table
pars.alpha = (-40:0.01:40)*pi/180;
pars.FyF = zeros(length(pars.alpha));
for i = 1:length(pars.alpha)
    pars.FyF(i) = Fiala(tire, Ca, mu, Fz, Fx, alpha(i));
end


end




