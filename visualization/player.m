function [M] = player(X, Y, a, b, phi, delta_f, sat_f, sat_r)
%PLAYER Plots the vehicle state over time
% Inputs:
%       X - global x position
%       Y - global y position
%       a - length from vehicle CG to front
%       b - length from vehicle CG to rear
%       phi - yaw angle
%       delta_f - front tire steer angle
%       sat_f - whether front tire is saturated, binary
%       sat_r - whether rear tire is saturated, binary
%
% Outputs:
%       M - movie frames (use movie(M) to view, mpgwrite(M) to save)

dsample = 10;
% Define the area to be recorded
rect = get(gcf,'Position');
rect(1:2) = [0 0];
sat_f(sat_f == 1) = 'r';
sat_f(sat_f == 0) = 'k';
sat_r(sat_r == 1) = 'r';
sat_r(sat_r == 0) = 'k';
% Generate and record the frames
for i = 1:length(X)/dsample
  %Plot the overall trajectory
  j = i*dsample;  %downsampling
  subplot(2,1,1);
  plot(X, Y,'g--'); hold on; %trajectory
  cg = [X(j),Y(j)];  %vehile center of gravity in longitudinal direction
  plot(cg(1), cg(2),'b.');  %plot current location
  axis([min(X) max(X) min(Y) max(Y)]);
  xlabel('X (m)'), ylabel('Y (m)'), title('Saturated Tire Model Vehicle');
  subplot(2,1,2); %zoomed in plot
  plot(X, Y,'g--'); hold on;  %trajectory
  
  %Plot body
  rear = [cg(1)-b*cos(phi(j)), cg(2)-b*sin(phi(j))];
  front = [cg(1)+a*cos(phi(j)), cg(2)+a*sin(phi(j))];
  line([rear(1) front(1)],[rear(2) front(2)],'Color','b','LineWidth',2);
  
  % Rear Tire
  d = 0.6; %arbitary tire diameter [m]
  backtire = [rear(1) - d/2*cos(phi(j)), rear(1) + d/2*cos(phi(j)), rear(2) - d/2*sin(phi(j)), rear(2) + d/2*sin(phi(j))];
  line([backtire(1) backtire(2)],[backtire(3) backtire(4)], 'Color',char(sat_r(j)),'LineWidth',8);
  
  % Front tire
  fronttire = [front(1) - d/2*cos(phi(j)+delta_f(j)), front(1) + d/2*cos(phi(j)+delta_f(j)), front(2) - d/2*sin(phi(j)+delta_f(j)), front(2) + d/2*sin(phi(j)+delta_f(j))];
  line([fronttire(1) fronttire(2)],[fronttire(3) fronttire(4)], 'Color',char(sat_f(j)),'LineWidth',8);
  
  %Zoomed in plot
  axis([X(j)-5*1.25 X(j)+4.25*1.25 Y(j)-1.5*1.25 Y(j)+1.5*1.25]);
  xlabel('X (m)'), ylabel('Y (m)');

  M(:,i) = getframe(gcf,rect); 
  clf; %clear the current figure  
end
% Play the movie
clf
N = 3; %number of times to loop
FPS = 20;  %frames per second
movie(gcf,M,N,FPS,rect)

end