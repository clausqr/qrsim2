function [Net_Forces, Net_Moments_CM] = Quadrotor_Aerodynamics(Atmosphere, Quadrotor, xi, Control, Aero_Model,perturbaciones)
global vh Vinf alfa

g   = Atmosphere.g;
rho = Atmosphere.rho;

Net_Forces = 0;
Net_Moments_CM = 0;

u     = xi(4,1);
v     = xi(5,1);
w     = xi(6,1);
phi   = xi(7,1);
theta = xi(8,1);
psi   = xi(9,1);
wx     = xi(10,1);
wy     = xi(11,1);
wz     = xi(12,1);

Vel_origin = [u v w] + cross(-Quadrotor.r_CG_O,[wx wy wz]);

prop_position = Quadrotor.prop_position;

prop_rotation_dir = Quadrotor.prop_rotation_dir;

M     = Quadrotor.Mass_prop.Mass;
A     = Quadrotor.DiskArea;
T_h   = M*g/4;
vh    = Quadrotor.vh;
k_th  = Quadrotor.k_th;

%%
% for prop = 1:4
%     % Compute Vinf and alfa of the propeller
%     pos = -Quadrotor.r_CG_O + prop_position(prop,:); % position relative to CG
%     Vel = [u v w] + cross(pos,[wx wy wz]);
%     Vinf = norm(Vel);
%     if Vinf == 0
%         alfa = pi/2;
%     else
%         alfa = pi/2 - acos(-Vel(1,3)/Vinf);
%     end
%     
%     % Thrust direction of propeller
%     thrust_direction = Quadrotor.(strcat('t', num2str(prop)));
%     
%     P = k_th*Control(prop);  % shaft power, assumed to be directly proportional to control signal
%     
%     switch Aero_Model
%         case 'Aero Model 1'
%             % Resuelvo ecuaciï¿½n 2.122 numï¿½ricamente para encontrar vi, y luego con vi y P en 2.121, calculo T.
%             % El problema con este modelo es que asume en una parte que T es
%             % constante igual a T_hover (o lo que es lo mismo: introduce vh).  Creo que
%             % esta manera de hacerlo tiene una inconsistencia por lo recien
%             % mencionado, que se radica precisamente en la frase que hay entre
%             % 2.121 y 2.122, donde dice que estamos en hover, pero en realidad
%             % NO estamos en hover.
%             
%             % The if statement is there to get a reasonable approximation of the vortex
%             % ring state.  Ver Figure 2.18 como para tener una idea de lo que
%             % estoy hablando.
%             OPTIONS = optimset('Display', 'off');
%             if -2 < Vel(1,3)/vh & Vel(1,3)/vh < -1
%                 'hello'
%                 vi = fsolve(@vi_fun,2*vh,OPTIONS);
%                 %     vi = fsolve(@vi_fun,2);
%             else
%                 vi = fsolve(@vi_fun,vh,OPTIONS);
%                 %     vi = fsolve(@vi_fun,2);
%             end
%             T = (P/(Vinf*sin(alfa) + vi))*thrust_direction;
%             
%         case 'Aero Model 2'
%             % Resuelvo el sistema no-lineal de ecuaciones compuesto por 2.110 (la primera de las 3
%             % igualdades que hay) y 2.121.  P, Vinf y alfa conocidos; T y vi
%             % incï¿½gnitas.  Para hacerlo hago sustituciï¿½n de vi en 2.121, y
%             % elevo al cuadrado para quitar la raï¿½z.  Esto es finalmente un
%             % polinomio de orden 6 en T.
%             
%             VV = Vinf; sa = sin(alfa); ca = cos(alfa);
%             a0 = P^4;
%             a1 = -2*P^3*VV*sa;
%             a2 = P^2*VV^2;
%             a3 = -2*P*VV*sa*VV^2*ca^2;
%             a4 = VV^4*sa^2*ca^2;
%             a6 = -1/(2*rho*A)^2;
%             poli = [a6 0 a4 a3 a2 a1 a0];
%             
%             if max(isnan(poli)) || max(isinf(poli))
%                 'YOU ARE SPINNING LIKE CRAZY!!!'
%                 T = [0 0 0];
%                 return
%             end
%             
%             T2 = roots(poli);
%             
%             T2 = min(T2((angle(T2) == 0 | angle(T2) == pi) & T2 > 0));
%             if isempty(T2)
%                 T2 = 0;
%             end
%             T = T2*thrust_direction/norm(thrust_direction);
%            
%     end
%     
%     % YAWING MOMENT of air on propeller
%     M = -Quadrotor.k_tm*norm(T)*prop_rotation_dir(prop)*[0 0 1];    % Yawing moment is approximately proportional to the thrust (k_tm), although there is a modelling error here
%     
%     Net_Forces = Net_Forces + T;
%     Net_Moments_CM = Net_Moments_CM + cross(pos, T) + M;
% end
%%
% TOTAL DRAG FORCE
% Cd = Quadrotor.Cd;
% if norm(Vel_origin) == 0
%     D = [0 0 0];
% else
%     Reference_Area = Quadrotor.width^2;
%     D = -Cd*0.5*rho*norm(Vel_origin)^2*(Vel_origin/norm(Vel_origin))*Reference_Area;
% end
%%


deltaR=perturbaciones;

ruido='gaussiano';

if strcmp(ruido,'gaussiano')
    deltaM=randn(3,1)*deltaR;
else
    deltaM=(rand(3,1)*2-1)*deltaR;
end


%Agregado cabeza: cero aerodinámica. el control pasa directo al torque


        L=Quadrotor.width/2;
        a=Quadrotor.k_tm;
        
        Fuerza_a_Torque_y_f=[ 0  -L   0   L;
                       -L   0   L   0;
                        a  -a   a  -a;
                       1/4 1/4 1/4 1/4];
                   
                   Torque_y_f = Fuerza_a_Torque_y_f*Control;
                   
Net_Forces = Torque_y_f(4)*[0 0 -1];

Perturbation_Moments = deltaM';
Control_Moments =  Torque_y_f(1:3)';
Net_Moments_CM = Perturbation_Moments + Control_Moments;

%%
% Net_Forces = Net_Forces + D;
% Net_Moments_CM = Net_Moments_CM + cross(-Quadrotor.r_CG_O, D);


% Function for Aero Model 1
function out = vi_fun(vi)
global vh Vinf alfa

out = vi - vh^2/(sqrt( (Vinf*cos(alfa))^2 + (Vinf*sin(alfa) + vi)^2));



