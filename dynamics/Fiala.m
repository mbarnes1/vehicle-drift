function [Fy, sat] = Fiala(tire, Ca, mu, Fz, Fx, alpha)
% Compute the fiala tire model for front or rear tire
% Input args:   tire - 'front' or 'rear'
%               Ca - tire cornering stiffness
%               mu - coefficient of friction
%               Fz - tire normal load
%               Fx - rear tire longitudinal force
%               alpha - tire slip angle in radians
% Output args:  Fy - lateral force
%               sat - whether tire is saturated, binary

z = tan(alpha);
if strcmp(tire, 'front')
    xi = 1;
elseif strcmp(tire, 'rear')
    xi = sqrt((mu*Fz)^2 - Fx^2)/(mu*Fz);
else
    error('Invalid tire input value');
end

tan_alpha_sl = 3*xi*mu*Fz/Ca;
alpha_sl = atan(tan_alpha_sl);

idx = abs(alpha) < alpha_sl;

Fy(idx) = -Ca*z(idx) + Ca^2/(3*xi*mu*Fz)*abs(z(idx)).*z(idx) - ...
    Ca^3/(27*xi^2*mu^2*Fz^2)*z(idx).^3;

Fy(~idx) = -xi*mu*Fz*sign(alpha(~idx));

sat = ~idx;

end

