global Atmosphere;
global Quadrotor;
global Aero_Model;
global t_final;
global t_step;
global X0;
global Control_Mode;
global CtrlParam;
global IC;
%%
%
% FlightFear run command:
% ./fgfs --fdm=network,localhost,5501,5502,5503 --fog-fastest --disable-clouds --in-air --enable-freeze --aircraft=QUADLSE
% 
% postes [37.62607004 -122.38053271]


%% Atomspheric parameters
Atmosphere.g    = 9.81;      % acceleration due to gravity (kg.m/s^2)
Atmosphere.rho  = 1.225;     % air density (kg/cm^3)
Atmosphere.Wind = [0 0 0];   % Constant wind (m/s) [not implemented]

%% Quadrotor parameters
Quadrotor.width  =   0.5%330;                   % distance between center or propellers (m)
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

Quadrotor.Mass_prop.Mass = 0.500; % mass of quadrotor (kg) original=0.55
% Quadrotor.Mass_prop.Ixx  = 1e-2;  % Moment of inertia in body x direction about CM  (kg.m^2)
% Quadrotor.Mass_prop.Iyy  = 1e-2;  % Moment of inertia in body y direction about CM  (kg.m^2)
% Quadrotor.Mass_prop.Izz  = 2e-2;  % Moment of inertia in body y direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Ixx  = 0.015;  % Moment of inertia in body x direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Iyy  = 0.015;  % Moment of inertia in body y direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Izz  = 0.025;  % Moment of inertia in body y direction about CM  (kg.m^2)
Quadrotor.Mass_prop.Ixy  = 0;     % Product of inertia in body x-y directions about CM (kg.m^2) 
Quadrotor.Mass_prop.Ixz  = 0;     % Product of inertia in body x-z directions about CM (kg.m^2)
Quadrotor.Mass_prop.Iyz  = 0;     % Product of inertia in body y-z directions about CM (kg.m^2)

Quadrotor.Mass_prop.I = [Quadrotor.Mass_prop.Ixx Quadrotor.Mass_prop.Ixy Quadrotor.Mass_prop.Ixz;
                         Quadrotor.Mass_prop.Ixy Quadrotor.Mass_prop.Iyy Quadrotor.Mass_prop.Iyz;
                         Quadrotor.Mass_prop.Ixz Quadrotor.Mass_prop.Iyz Quadrotor.Mass_prop.Izz];

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
t_step = 0.01;          % Simulation time step (sec)
Aero_Model = 'Aero Model 2'; % Selects aerodynamic model

%% Control Parameters
CtrlParam.ControllerSampleTime = 0.050;        % time between control command refresh (sec) %%%%OBSOLETO!!!! 
%Control_Mode    = 'Hover PID';
Control_Mode    =   'control [2]';

CtrlParam.kappa_max =   0.1 *pi/180;
CtrlParam.chi_max   =   0.1 *pi/180;
CtrlParam.deltaR    =   0.5;
CtrlParam.Omega_MAX      =   120 *pi/180;
CtrlParam.Omegad_MAX     =   120 *pi/180;
CtrlParam.Omegad_dot_MAX =   180 *pi/180;



%% Initial conditions
% All expressed in SIMULATION frames of reference
Pos_x = 0;   % (m)
Pos_y = 0;   % (m)
Pos_z = -4;  % (m)

Roll  = 1;  % (deg)
Pitch = 0;   % (deg)
Yaw   = 0;   % (deg)

Vel_x = 0;   % (m/s)
Vel_y = 0;   % (m/s)
Vel_z = 0;   % (m/s)

AngVel_x = 0;  % (deg/s)
AngVel_y = 0;   % (deg/s)
AngVel_z = 1;   % (deg/s)

Velocity = 0; % (m/s)
alfa = 0;     % (deg)
beta = 0;     % (deg)

IC.position         = [Pos_x, Pos_y, Pos_z];            % XYZ position of origin of B w.r.t. I, expressed in I (m)
IC.velocity         = [Vel_x, Vel_y, Vel_z];            % Velocity of origin of B w.r.t. I (m/s). ICVelOption selects in which frame the velocity is expressed  
IC.orientation      = [Roll, Pitch, Yaw];               % Roll, pitch and Yaw Euler angles, of B w.r.t. I (deg)
IC.angular_velocity = [AngVel_x, AngVel_y, AngVel_z];   % Angular velocity of B w.r.t. I, expressed in I (deg/s)
IC.LatLonAltOnEarth = [37.626537 -122.379584 7224];     % Geodetic latitude, longitude and altitude  (for FliqhtGear simualtion)                            
IC.Vel  = Velocity;  % Modulus of velocity of origin of B w.r.t. I
IC.alfa = alfa;      % angle of attack (deg)
IC.beta = beta;      % side slip angle (deg)

IC.VelOption        = 'Inertial';                       % Choose in what way you want to define IC velocity - 'Inertial': velocity vector expressed in Inertial frame, 'Body': velocity vector expressed in Body frame, 'WindAngles': express it in terms of absolute velocity and wind angles (Vinf, alfa, beta)

X0 = InitialCondition(IC,Quadrotor.r_CG_O);
