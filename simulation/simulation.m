%% simulation.m
clear; close all; clc;
addpath(genpath('../'));

%% Get initial states, parameters (controller gains, etc.)
pars = GetParameters();

% ts = linspace(pars.t0, pars.T, pars.dt);
ts = pars.t0:pars.dt:pars.T;
nsteps = length(ts);

%% Define the initial state x and control inputs u
x = pars.x0;    % [beta; r; U_x]
u = pars.u0;    % [delta; F_xR]
vs = pars.vs0;  % [X;Y;Theta]

%% Store state, control inputs
VS = NaN(3,nsteps);
dX  = NaN(3,nsteps);
X   = NaN(3,nsteps);
U   = NaN(2,nsteps);

%% Run simulation
for t = 1:nsteps
    if t == 6133
        aaa = 0;
    end
    %% Get control inputs
    u_plus = Controller(x,u,pars); % Compute control inputs u

    %% Compute dynamics with ode45
%     u_plus = pars.u0;
    
    [~, x_plus] = ode45(@(t,x) Dynamics(x,u_plus,pars),[0 pars.dt],x);
    x_plus = x_plus(end,:)';
    
    %% Compute dynamics with euler integration
%     dx_plus = Dynamics(x,u_plus,pars); % Compute state x after control inputs u
%     x_plus = IntegrateDynamics(dx_plus,x,pars.dt);

    %% Compute vehicle position
    vs_plus = State(vs, x_plus, pars.dt);
    
    %% Update the states
    x = x_plus;
    u = u_plus;
    vs = vs_plus;

    %% Save the states for plotting
    X(:,t) = x_plus;
    U(:,t) = u_plus;
    VS(:,t) = vs_plus;

    fprintf('t = %.0f\n', t);
end

%% Plot resulting Beta trajectory
figure; 
subplot(3,1,1)
plot(ts, X(1,:)*180/pi, 'b'); hold on;
plot(ts, ones(nsteps, 1)*pars.beta_eq*180/pi, 'g');
xlabel('Time (s)'); ylabel('\beta (degrees)');

subplot(3,1,2)
plot(ts, X(2,:)*180/pi, 'b'); hold on;
plot(ts, ones(nsteps, 1)*pars.r_eq*180/pi, 'g');
xlabel('Time (s)'); ylabel('r (degrees / sec)');

subplot(3,1,3)
plot(ts, X(3,:), 'b'); hold on;
plot(ts, ones(nsteps, 1)*pars.Ux_eq, 'g');
xlabel('Time (s)'); ylabel('U_X (m/s)');

figure; 
subplot(2,1,1)
plot(ts, U(1,:)*180/pi, 'b'); hold on;
xlabel('Time (s)'); ylabel('\delta_{des} (degrees)');

subplot(2,1,2)
plot(ts, U(2,:), 'b'); hold on;
xlabel('Time (s)'); ylabel('F_{X}R (N)');

% figure
% plot(ts, (X(1,:) - U(1,:))*180/pi)

%% Visualize the trajectory

% player(VS(1,:), VS(2,:), pars.a, pars.b, VS(3,:), U(1,:))


















