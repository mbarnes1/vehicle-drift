function Fy = Fiala(tire, Ca, mu, Fz, Fx, alpha)
% Compute the fiala tire model for front or rear tire
% Input args:   tire - 'front' or 'rear'
%               Ca - tire cornering stiffness
%               mu - coefficient of friction
%               Fz - tire normal load
%               Fx - rear tire longitudinal force
%               alpha - tire slip angle in radians

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

if (abs(alpha) < alpha_sl)
    Fy = -Ca*z + Ca^2/(3*xi*mu*Fz)*abs(z)*z - Ca^3/(27*xi^2*mu^2*Fz^2)*z^3;
else
    Fy = -xi*mu*Fz*sign(alpha);
end

end

