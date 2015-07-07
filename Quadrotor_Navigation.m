function [R_est, Omega_est] = Quadrotor_Navigation(CtrlParam, xi)


x     = xi(1,1);
y     = xi(2,1);
z     = xi(3,1);
u     = xi(4,1);
v     = xi(5,1);
w     = xi(6,1);
phi   = xi(7,1);
theta = xi(8,1);
psi   = xi(9,1);
p     = xi(10,1);
q     = xi(11,1);
r     = xi(12,1);


R = angle2dcm(psi, theta, phi)';

%%
kappa_max = CtrlParam.kappa_max;
chi_max   = CtrlParam.chi_max;

ruido='gaussiano';

if strcmp(ruido, 'gaussiano')
    z=randn(3,1)*kappa_max;
    chi=randn(3,1)*chi_max;
elseif strcmp(ruido, 'uniforme')
    z=(rand(3,1)*2-1)*kappa_max;
    chi=(rand(3,1)*2-1)*chi_max;
else
    z=[0 0 0]';
    chi=[0 0 0]';
end
%%
z_hat=[0 -z(3) z(2); z(3) 0 -z(1); -z(2) z(1) 0];
Z=expm(z_hat);
R_est=R*Z;

R_est=R_est(:);
Omega_est = [p q r]'+chi;


