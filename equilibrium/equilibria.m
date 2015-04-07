% Vehicle equilibrium analysis

%% Setup
close all; clear; clc;

%% Solve for equilibrium points

% Define equilibrium Ux and delta, and output container
Ux_eq = 8;                  % m / s
delta_deg1 = -20:12;        % deg
delta_deg2 = -12:20;        % deg
delta_deg3 = -12:12;        % deg
delta1 = delta_deg1*pi/180; % rad
delta2 = delta_deg2*pi/180; % rad
delta3 = delta_deg3*pi/180; % rad
N1 = numel(delta1);         % number of points
N2 = numel(delta2);         % number of points
N3 = numel(delta3);         % number of points

% Output containers
eq_points1 = zeros(3,N1);
eq_points2 = zeros(3,N2);
eq_points3 = zeros(3,N3);
x0_1 = zeros(3,N1);
x0_2 = zeros(3,N2);
x0_3 = zeros(3,N3);

options = optimoptions('fsolve');

for i = 1:N1
    % Define equilibrium dynamics function for these Ux and delta
    eq_dynamics1 = @(x) dynamics(Ux_eq, delta1(i), x(1), x(2), x(3));
    
    % Define initialization point
    x0_1(:,i) = [0.6;                           % r - yaw rate (rad / s)
%                 -1000/15*delta_deg1(i)+1500;    % FxR - rear longitudinal force (N)
                1500;
                (delta_deg1(i)-10)*pi/180];     % Beta - sideslip angle in radians (rad)
    
    % Solve for equilibrium point
    eq_points1(:,i) = fsolve(eq_dynamics1, x0_1(:,i), options);
end

for i = 1:N2
    % Define equilibrium dynamics function for these Ux and delta
    eq_dynamics2 = @(x) dynamics(Ux_eq, delta2(i), x(1), x(2), x(3));
    
    % Define initialization point
    x0_2(:,i) = [-0.6;                      % r - yaw rate (rad / s)
%                 1000/15*delta_deg2(i)+1500; % FxR - rear longitudinal force (N)
                1500;
                (delta_deg2(i)+10)*pi/180]; % Beta - sideslip angle in radians (rad)
    
    % Solve for equilibrium point
    eq_points2(:,i) = fsolve(eq_dynamics2, x0_2(:,i), options);
end

for i = 1:N3
    % Define equilibrium dynamics function for these Ux and delta
    eq_dynamics3 = @(x) dynamics(Ux_eq, delta3(i), x(1), x(2), x(3));
    
    % Define initialization point
    x0_3(:,i) = [0.05*delta_deg3(i);    % r - yaw rate (rad / s)
%                 3*delta_deg3(i)^2;      % FxR - rear longitudinal force (N)
                0;
                0*pi/180];              % Beta - sideslip angle in radians (rad)
    
    % Solve for equilibrium point
    options = optimoptions('fsolve', 'MaxFunEvals', 1000, 'MaxIter', 1000, ...
                'TolFun', 1e-3, 'TolX', 1e-3);
    eq_points3(:,i) = fsolve(eq_dynamics3,  x0_3(:,i), options);
end


%% Compute other forces


%% Plot equilibrium points
FS = 14;

h1 = figure; hold on;
% set(h1, 'defaulttextinterpreter','latex');
% plot(delta_deg1, x0_1(1,:), 'r^', delta_deg2, x0_2(1,:), 'r^');
% plot(delta_deg3, x0_3(1,:), 'r*');
plot(delta_deg1, eq_points1(1,:), 'b^');
plot(delta_deg2, eq_points2(1,:), 'b^');
plot(delta_deg3, eq_points3(1,:), 'b*');
xlabel('\delta^{eq}(deg)','FontSize', FS);
ylabel('r^{eq} (rad/s)','FontSize', FS);

h2 = figure; hold on;
% set(h2, 'defaulttextinterpreter','latex');
% plot(delta_deg1, x0_1(2,:), 'r^', delta_deg2, x0_2(2,:), 'r^');
% plot(delta_deg3, x0_3(2,:), 'r*');
plot(delta_deg1, eq_points1(2,:), 'b^');
plot(delta_deg2, eq_points2(2,:), 'b^');
plot(delta_deg3, eq_points3(2,:), 'b*');
xlabel('\delta^{eq}(deg)','FontSize', FS);
ylabel('F_{xR}^{eq} (N)','FontSize', FS);

h3 = figure; hold on;
% set(h3, 'defaulttextinterpreter','latex');
% plot(delta_deg1, x0_1(3,:)*180/pi, 'r^', delta_deg2, x0_2(3,:)*180/pi, 'r^');
% plot(delta_deg3, x0_3(3,:)*180/pi, 'r*');
plot(delta_deg1, eq_points1(3,:)*180/pi, 'b^');
plot(delta_deg2, eq_points2(3,:)*180/pi, 'b^');
plot(delta_deg3, eq_points3(3,:)*180/pi, 'b*');
xlabel('\delta^{eq}(deg)','FontSize', FS);
ylabel('\beta^{eq} (deg)','FontSize', FS);

saveas(h1, 'r', 'png');
saveas(h2, 'FxR', 'png');
saveas(h3, 'beta', 'png');























