function [Control, errores] = Quadrotor_Controller(Atmosphere, Quadrotor, Control_Mode, CtrlParam, R, Omega,x,v)
% Reading current state
%    X - state vector of the rigid body
%         X(1,1)  = x; X-position of CM w.r.t. I, expressed in I
%         X(2,1)  = y; Y-position of CM w.r.t. I, expressed in I
%         X(3,1)  = z; Y-position of CM w.r.t. I, expressed in I
%         X(4,1)  = u; X-velocity of CM w.r.t. I, expressed in B
%         X(5,1)  = v; Y-velocity of CM w.r.t. I, expressed in B
%         X(6,1)  = w; Z-velocity of CM w.r.t. I, expressed in B
%         X(7,1)  = phi;   Standard Roll Euler angle of B w.r.t I
%         X(8,1)  = theta; Standard Pitch Euler angle of B w.r.t I
%         X(9,1)  = psi;   Standard Yaw Euler angle of B w.r.t I
%         X(10,1) = wx;    X-angular velocity of B frame w.r.t. I, expressed in B
%         X(11,1) = wy;    Y-angular velocity of B frame w.r.t. I, expressed in B
%         X(12,1) = wz;    Z-angular velocity of B frame w.r.t. I, expressed in B

% x     = xi(1,1);
% y     = xi(2,1);
% z     = xi(3,1);
% u     = xi(4,1);
% v     = xi(5,1);
% w     = xi(6,1);
% phi   = xi(7,1);
% theta = xi(8,1);
% psi   = xi(9,1);
% p     = xi(10,1);
% q     = xi(11,1);
% r     = xi(12,1);


%%
persistent Omegad_dot;
persistent Omegad;
persistent t_since_last_call;
persistent M;
persistent throttle;
persistent Fuerzas;


if (isempty(t_since_last_call)||(t_since_last_call<0.00000001));
    
    t_since_last_call = 100;
end
if isempty(Omegad)
    throttle=0;
    Omegad = [0 0 0]';
end
if isempty(Omegad_dot)
    Omegad_dot = [0 0 0]';
end

persistent t_old;
persistent t_now;

%% Leemos parámetros
% Gravity
g = Atmosphere.g;

% Reading Quadrotor properties
m   = Quadrotor.Mass_prop.Mass;             % kg
Ixx = Quadrotor.Mass_prop.Ixx;              % kg.m^2
Iyy = Quadrotor.Mass_prop.Iyy;              % kg.m^2
Izz = Quadrotor.Mass_prop.Izz;              % kg.m^2
Ixy = Quadrotor.Mass_prop.Ixy;              % kg.m^2
Ixz = Quadrotor.Mass_prop.Ixz;              % kg.m^2
Iyz = Quadrotor.Mass_prop.Iyz;              % kg.m^2
Index_roll  = Quadrotor.Index_roll;
Index_pitch = Quadrotor.Index_pitch;
Index_yaw   = Quadrotor.Index_yaw;

% Control parameters

deltaR=CtrlParam.deltaR;

global t_step;
deltaT     = t_step;

t_old=t_now;
t_now=CtrlParam.t_now;

deltaT=(t_now - t_old)+0.000001;
%   deltaT=0.01;

%% Control Policy


switch Control_Mode
    case 'Hover PID'
        % Calculate PID coefficients
        kp_z  = wn_z^2;         kd_z = 2*zeta_z*wn_z;
        kp_phi  = wn_phi^2;     kd_phi = 2*zeta_phi*wn_phi;
        kp_theta  = wn_theta^2; kd_theta = 2*zeta_theta*wn_theta;
        kp_psi  = wn_psi^2;   kd_psi = 2*zeta_psi*wn_psi;
        
        % Verical control
        z_des  = CtrlParam.z_des;
        TotalThrust = m*g + m*kd_z*w + m*kp_z*(z - z_des);
        % Roll control
        Mx = -Ixx*kd_phi*p - Ixx*kp_phi*phi;
        % Pitch control
        theta_des=CtrlParam.theta_des;
        My = -Iyy*kd_theta*q - Iyy*kp_theta*(theta-theta_des);
        % Yaw control
        psi_des  = CtrlParam.psi_des;
        Mz = -Izz*kd_psi*r - Izz*kp_psi*(psi - psi_des);
        
        % Thrust desired for each propeller
        Thrust = TotalThrust*ones(1,4)/4 + Mx.*Index_roll + My.*Index_pitch + Mz.*Index_yaw/Quadrotor.k_tm;
        
        % Control input for each propeller (convert thrust desired to
        % control input) This has a modelling error, becuase it doesn't
        % take into account the changes in thrust due to relative velocity (see Quadrotor_Aerodynamics)
        Control = Thrust*Quadrotor.vh/Quadrotor.k_th;
        Control = min(Control,1); Control = max(Control, 0);
    case 'Constant Input'
        Control = ones(1,4)*CtrlParam.ConstantControl;
    case 'control [2]'
        
        %angle2cm default = 'ZYX'
        % psi = yaw
        % theta = pitch
        % phi = roll
        %Rd = actitud de referencia
        J = Quadrotor.Mass_prop.I;
        
        psi_des = CtrlParam.psi_des;
        theta_des = CtrlParam.theta_des;
        phi_des = CtrlParam.phi_des;
        
        Rd = angle2dcm(psi_des, theta_des, phi_des)';
        
        %Forma muy cabeza de calcular Rd_dot
        
        psi_des_dot = CtrlParam.psi_des_dot;
        theta_des_dot = CtrlParam.theta_des_dot;
        phi_des_dot = CtrlParam.phi_des_dot;
        
        
        %         if isempty(deltaT)
        %             Rd_dot=zeros(3);
        %         else
        deltaT2=0.00005;
        Rd_F=angle2dcm(...
            psi_des+psi_des_dot*deltaT2,...
            theta_des+theta_des_dot*deltaT2,...
            phi_des+phi_des_dot*deltaT2)';
        Rd_dot = (Rd_F - Rd)*1/deltaT2;
        %         end
        
        
        
        
        % Leemos las ganancias
        k_R     = CtrlParam.k_R;
        k_Omega = CtrlParam.k_Omega;
        c2      = CtrlParam.c2;
        
        
        %       R = angle2dcm(psi, theta, phi)';
        
        % e_R
        e_R_hat = 1/2*(Rd'*R - R'*Rd);
        e_R = [e_R_hat(3,2) e_R_hat(1,3) e_R_hat(2,1)]';
        
        
        %Omega = [p q r]';
        Omega_hat = [ 0        -Omega(3)  Omega(2);
            Omega(3)  0        -Omega(1);
            -Omega(2)  Omega(1)  0];
        
        Omegad_hat = Rd'*Rd_dot;
        
        Omegad_old = Omegad;
        Omegad = [-Omegad_hat(2,3) Omegad_hat(1,3) -Omegad_hat(1,2)]';
        
        if isempty(deltaT)
            %wtf?
            deltaT;
            Omegad_dot=[0 0 0]';
        else
            Omegad_dot = (Omegad - Omegad_old)/deltaT/2;
        end
        
        
        % e_Omega
        e_Omega = Omega - R'*Rd*Omegad;
        
        
        if t_since_last_call>CtrlParam.ControllerSampleTime
            
            k_max   =  CtrlParam.kappa_max;
            X_max     =  CtrlParam.chi_max;
            k_Om      =  k_Omega;
            Om_max    =  CtrlParam.Omega_MAX;
            Om_d_max  =  CtrlParam.Omegad_MAX;
            dOm_d_max =  CtrlParam.Omegad_dot_MAX;
            lam_m = min([Ixx Iyy Izz]);
            lam_M = max([Ixx Iyy Izz]);
            
            deltaE = 2*sin(k_max/2)*...
            (k_R+k_Om*Om_d_max+lam_M*(Om_max*Om_d_max+dOm_d_max))+...
            (lam_M*(Om_max+Om_d_max)+k_Om)*k_max;


      a=2*sin(k_max/2)*(Om_d_max+c2/lam_m)*X_max;
      
            alfa_d = (-Omega_hat*R'*Rd*Omegad + R'*Rd*Omegad_dot);
            
            e_A = e_Omega+c2*J^(-1)*e_R;
            
            mu  = -(deltaE+deltaR)*e_A./((norm(e_A)+a));
            
            M = -k_R*e_R...
                -k_Omega*e_Omega...
                +Omega_hat*J*Omega...
                +J*alfa_d...
                +mu;
        %    t_since_last_call
            t_since_last_call = 0;
            
            
            
            %el control de posicion mas cabeza de la historia...
            cos_eje_z=abs([0 0 1]*Rd(:,3));
            f = m*g/max([cos_eje_z 0.1])*0.95;%(0.8*(x(3)+4)+0.4*v(3)); 
            f_max=40;
            f = f.*(f>0).*(f<f_max)+f_max.*(f>=f_max);
            % Bien cabeza...
            throttle=CtrlParam.throttle+f;
            L=Quadrotor.width/2;
            b=Quadrotor.k_tm;
                                    
            Fuerza_a_Torque_y_f= [ 0  -L   0   L;
                                 -L   0   L   0;
                                   b  -b   b  -b;
                                  1/4 1/4 1/4 1/4];
            
            TorquesAFuerza =  Fuerza_a_Torque_y_f^-1;%\eye(4);
            Fuerzas = TorquesAFuerza*[M' throttle]';
            
        end
        
        Control = Fuerzas;
    case 'Su controlador preferido aquí'
        
        
end
errores=[asind(norm(e_R)) norm(e_Omega)*180/pi];
t_since_last_call = t_since_last_call + deltaT;


