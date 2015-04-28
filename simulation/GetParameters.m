function pars = GetParameters()

%% Timing
pars.t0 = 0;            % seconds
pars.T = 10;            % seconds
pars.dt = 1e-2;         % seconds

%% Initial state and control
pars.x0 = [(-50.44)*pi/180;         % Beta
            0.600;                  % r
            8];                     % Ux
pars.u0 = [-12*pi/180;2293];
pars.vs0 = [0;0;0];             % Initial vehicle position and orientation

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
pars.delta_eq   = -12*pi/180;   % rad
pars.Ux_eq      = 8;            % m / s

pars.beta_eq    = -20.44*pi/180;% rad
% pars.beta_eq    = -21.35*pi/180;% rad
pars.r_eq       = 0.600;        % rad / s
% pars.r_eq       = 0.5061;        % rad / s
pars.FxR_eq     = 2293;         % N
% pars.FxR_eq     = 2290;         % N
pars.FyF_eq     = 3807;         % N
pars.FyR_eq     = 4469;         % N

pars.x_eq = [pars.beta_eq; pars.r_eq; pars.Ux_eq];

%% Define control limits
pars.FxR_max = pars.mu*pars.FzR;

%% Controller parameters
pars.K_beta = 2;
pars.K_r    = 4;
pars.K_Ux   = 0.846;

% pars.K_beta = -3060;
% pars.K_r    = 6717.4;
% pars.K_Ux   = -602.5;

% pars.K_beta = -3060;
% pars.K_r    = 6717.4;
% pars.K_Ux   = -602.5;


end




