%% Get initial states, parameters (controller gains, etc.)
pars = GetParameters();
T = pars.T;
t0 = pars.t0;
dt = pars.dt;

ts = linspace(t0,T,dt);
nsteps = length(ts);

%% Define the initial state x and control inputs u
x = x_0;                                % [beta, r, U_x]
u = u_0;                                % [F_xR, delta]

%% Store state, control inputs
dX = NaN(3,nsteps);
X = NaN(3,nsteps);
U = NaN(2,nsteps);

%% Run simulation!!
for t = 1:nsteps

    %% Get control inputs
    u_plus = Controller(x,u,pars); % Compute control inputs u

    %% Compute dynamics
    dx_plus = Dynamics(x,u_plus,pars); % Compute state x after control inputs u
    
    x_plus = IntegrateDynamics(dx_plus,x,dt);

    %% Display
    DisplayCar(x_plus,u_plus,pars);
    
    %% Update and save the states
    dx = dx_plus;
    x = x_plus;
    u = u_plus;

    dX(:,t) = dx_plus;
    X(:,t) = x_plus;
    U(:,t) = t_plus;

end
