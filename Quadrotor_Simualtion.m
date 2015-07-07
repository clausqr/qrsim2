%% QUADROTOR SIMULATION
% Author: Roberto A. Bunge, Ph.D. Candidate 
% Department of Aeronautics and Astronautics
% Stanford University
% email address: rbunge@stanford.edu 
% August 2011; Last revision: 18-Sep-2011

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% N O T E S %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All quantities should be in SI units.
%
% Frames of Reference:
%  SIMULATION    
%   Body fixed frame (B): has X in forward direction, Y to the right, and Z
%                         pointing down. Origin of B coincides with geometric center of
%                         Quadrotor.
%   Inertial frame (I): fixed to earth.  When all flight variables are zero, B
%                       and I coincide.
%  ANIMATION
%   Body fixed frame: has X in forward direction, Y up, and Z pointing to
%                     the right. Origin coincides with geometric center of
%                     Quadrotor.
%   Inertial frame: fixed to the earth. When all flight variables are zero, B
%                   and I coincide.
%
% Drag Coefficient: the simulation (especially the translational motion) is quite sensitive to the value chosen
%                   for Cd.  Often Cd is around 0.5 and 1.5
%%%%%%%%%%%%%%%%%%%%%%%%%

clear T X C
close all
%% Path Settings
% path(path,'/Users/Robbie/Documents/AA Research/Flight Simulation Codes/Aircraft input files')
% path(path,'/Users/Robbie/Documents/AA Research/Flight Simulation Codes')
% path(path,'/Users/Robbie/Documents/AA Research/Flight Simulation Codes/Plotting')
% path(path,'/Users/Robbie/Documents/AA Research/QuadAir/QuadAir 1.0')
% path(path,'/Users/Robbie/Documents/AA Research/Flight Simulation Codes/Simulation 1')
% path(path,'/Applications/MATLAB_R2010aSV.app/toolbox/aero/aero/@Aero/@Node/private')

%% Open Virtual Reality Animation World
if exist('QuadAnimation')
else
    QuadAnimation = Aero.VirtualRealityAnimation;
    QuadAnimation.VRWorldFilename = 'Quadrotor_World.wrl';
    QuadAnimation.initialize();
    fig = QuadAnimation.VRFigure;
    set(fig, 'Viewpoint', 'Standard View');
end

%% Atomspheric parameters
Atmosphere.g    = 9.81;      % acceleration due to gravity (kg.m/s^2)
Atmosphere.rho  = 1.225;     % air density (kg/cm^3)
Atmosphere.Wind = [0 0 0];   % Constant wind (m/s) [not implemented]

%% Quadrotor parameters
Quadrotor.width  =   0.330;                   % distance between center or propellers (m)
Quadrotor.r_CG_O =   [0 0 0];                 % position of CG relative to origin of B, expressed in B (m).
Quadrotor.prop_position = [ 1   1  0 
                           -1   1  0 
                           -1  -1  0 
                            1  -1  0]*Quadrotor.width/2;  % position of each propeller w.r.t to origin of B, expressed in B
Quadrotor.prop_rotation_dir = [-1 1 -1 1];      % positive aligned with body Z axis (i.e. clockwise from the top)
Quadrotor.t1     =  -[0 0 1];                    % unit vector aligned with nominal trhust direction of rotor 1
Quadrotor.t2     =  -[0 0 1];                    % unit vector aligned with nominal trhust direction of rotor 2
Quadrotor.t3     =  -[0 0 1];                    % unit vector aligned with nominal trhust direction of rotor 3
Quadrotor.t4     =  -[0 0 1];                    % unit vector aligned with nominal trhust direction of rotor 4

Quadrotor.Mass_prop.Mass = 0.550; % mass of quadrotor (kg)
Quadrotor.Mass_prop.Ixx  = 1e-2;  % Moment of inertia in body x direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Iyy  = 1e-2;  % Moment of inertia in body y direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Izz  = 1e-2;  % Moment of inertia in body y direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Ixy  = 0;     % Product of inertia in body x-y directions about CM (kg.m^2) 
Quadrotor.Mass_prop.Ixz  = 0;     % Product of inertia in body x-z directions about CM (kg.m^2)
Quadrotor.Mass_prop.Iyz  = 0;     % Product of inertia in body y-z directions about CM (kg.m^2)

Quadrotor.DiskDiameter = 0.203;   % Diameter of propeller disk (m)
Quadrotor.DiskArea     = Quadrotor.DiskDiameter^2/4;

Quadrotor.Hover_Command = 0.25;      % Nominal average command given to propellers while in hover (duty cycle, 0 to 1)

Quadrotor.vh   = sqrt((Quadrotor.Mass_prop.Mass*Atmosphere.g/4)/(2*Atmosphere.rho*Quadrotor.DiskArea));  % propeller hower induced velocity (m/s).  See Leishmann's book.
Quadrotor.k_th = Quadrotor.Mass_prop.Mass*Atmosphere.g*Quadrotor.vh/(Quadrotor.Hover_Command*4);         % Constant relating Motor command to power
Quadrotor.k_tm = 1;                  % Constant relating thrust to torque
Quadrotor.Cd   = 0.5;                % Drag coefficient for whole quadrotor. Drag = Cd*0.5*rho*V^2*ReferenceArea

Quadrotor.Index_roll  = [-1  -1   1   1];   % Propeller combination used for roll command in the controller
Quadrotor.Index_pitch = [ 1  -1  -1   1];   % Propeller combination used for pitch command in the controller
Quadrotor.Index_yaw   = [ 1  -1   1  -1];   % Propeller combination used for yaw command in the controller



%% Simulation parameters
t_final = 15;
t_step = 0.05;          % Simulation time step (sec)
Aero_Model = 'Aero Model 2'; % Selects aerodynamic model

%% Control Parameters
Control_Period  = 0.05;        % time between control command refresh (sec)
Control_Mode    = 'Hover PID';
CtrlParam.ConstantControl = 1;
CtrlParam.z_des      = -6;
CtrlParam.psi_des    = 0;

CtrlParam.wn_z       = 1;
CtrlParam.zeta_z     = 0.5;

CtrlParam.wn_phi     = 5;
CtrlParam.zeta_phi   = 1;

CtrlParam.wn_theta   = 5;
CtrlParam.zeta_theta = 1;

CtrlParam.wn_psi   = 2;
CtrlParam.zeta_psi = 1;


%% Initial conditions
% All expressed in SIMULATION frames of reference
Pos_x = 0;   % (m)
Pos_y = 0;   % (m)
Pos_z = -4;  % (m)

Roll  = 70;  % (deg)
Pitch = 0;   % (deg)
Yaw   = 0;   % (deg)

Vel_x = 0;   % (m/s)
Vel_y = 0;   % (m/s)
Vel_z = 0;   % (m/s)

AngVel_x = 0;  % (deg/s)
AngVel_y = 0;   % (deg/s)
AngVel_z = 0;   % (deg/s)

Velocity = 0; % (m/s)
alfa = 0;     % (deg)
beta = 0;     % (deg)

IC.position         = [Pos_x, Pos_y, Pos_z];            % XYZ position of origin of B w.r.t. I, expressed in I (m)
IC.velocity         = [Vel_x, Vel_y, Vel_z];            % Velocity of origin of B w.r.t. I (m/s). ICVelOption selects in which frame the velocity is expressed  
IC.orientation      = [Roll, Pitch, Yaw];               % Roll, pitch and Yaw Euler angles, of B w.r.t. I (deg)
IC.angular_velocity = [AngVel_x, AngVel_y, AngVel_z];   % Angular velocity of B w.r.t. I, expressed in I (deg/s)
IC.LatLonAltOnEarth = [37.665543 -122.480847 7224];     % Geodetic latitude, longitude and altitude  (for FliqhtGear simualtion)                            
IC.Vel  = Velocity;  % Modulus of velocity of origin of B w.r.t. I
IC.alfa = alfa;      % angle of attack (deg)
IC.beta = beta;      % side slip angle (deg)

IC.VelOption        = 'Inertial';                       % Choose in what way you want to define IC velocity - 'Inertial': velocity vector expressed in Inertial frame, 'Body': velocity vector expressed in Body frame, 'WindAngles': express it in terms of absolute velocity and wind angles (Vinf, alfa, beta)

X0 = InitialCondition(IC,Quadrotor.r_CG_O);

%% START SIMULATION
% Initialization
N=t_final/t_step+1; dt = t_step; T_last_control = -Control_Period;
X = zeros(length(X0),N); X(:,1) = X0'; T=zeros(N,1); C=zeros(N,4);

% Main integration loop
for i = 2:N
    
    % Get current vehicle state
    xi = X(:,i-1);
    if max(isnan(xi))  % Check that simulation didn't go bad
        X(:,i-1) = zeros(length(X0),1);
        break
    end
  
    %girarara?
   % CtrlParam.psi_des    = -T_last_control*pi/180*100;    %36 grados/s
    %rebotara?
  %  CtrlParam.z_des      = -6-5*sin(2*pi*T_last_control/2.5);
    
    % Simulaci?n de Ruido
    
    % Estimaci?n
    
    
    % Control
    if T(i-1) - T_last_control >= Control_Period
        Control = Quadrotor_Controller(Atmosphere, Quadrotor, Control_Mode, CtrlParam, xi);
        T_last_control = T(i-1);
    end
    
    % Gravity force
    [Gravity_Force] = Gravity_Forces(Atmosphere, Quadrotor, xi);
    
    % Aerodynamic forces and moments
    [Aero_Forces, Aero_Moments_CM] = Quadrotor_Aerodynamics(Atmosphere, Quadrotor, xi, Control, Aero_Model);
    
    % Net forces and moments
    Net_Forces      = Gravity_Force + Aero_Forces;
    Net_Moments_CM  = zeros(1,3) + Aero_Moments_CM;

    % Propagate rigid body dynamics forward in time, with RK4 integration of the equations of motion
    Mass_prop = Quadrotor.Mass_prop;
    F(:,1) = dt*feval('RigidBodyDynamics', xi, Quadrotor.Mass_prop, Net_Forces, Net_Moments_CM);
    F(:,2) = dt*feval('RigidBodyDynamics', xi+0.5*F(:,1), Quadrotor.Mass_prop, Net_Forces, Net_Moments_CM);
    F(:,3) = dt*feval('RigidBodyDynamics', xi+0.5*F(:,2), Quadrotor.Mass_prop, Net_Forces, Net_Moments_CM);
    F(:,4) = dt*feval('RigidBodyDynamics', xi+F(:,3), Quadrotor.Mass_prop, Net_Forces, Net_Moments_CM);
    X(:,i) = xi + (1/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4));  
    T(i) = T(i-1)+dt;
    C(i,:) = Control;
end

%% Plot outputs
%Trayectory_Plotter_2(X, T, C, 0, 'Quadrotor')

%% Animation
Quad = QuadAnimation.nodes{8}.VRNode;
False_Quad = QuadAnimation.nodes{5}.VRNode;
shadow = QuadAnimation.nodes{2}.VRNode;

playVRML(T,X,Quadrotor.r_CG_O,QuadAnimation,Quad,False_Quad,shadow)









