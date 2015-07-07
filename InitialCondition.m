function X0 = InitialCondition(IC, r_CG_O)
% I = Inertial frame
% B = Body fixed frame

% IC.position         = [Pos_x, Pos_y, Pos_z];            % XYZ position of origin of B w.r.t. I, expressed in I (m)
% IC.orientation      = [Roll, Pitch, Yaw];               % Roll, pitch and Yaw Euler angles, of B w.r.t. I (deg)
% IC.velocity         = [Vel_x, Vel_y, Vel_z];            % Velocity of origin of B w.r.t. I (m/s). ICVelOption selects in which frame the velocity is expressed. ICVelOption selects in which frame the velocity is expressed  
% IC.angular_velocity = [AngVel_x, AngVel_y, AngVel_z];   % Angular velocity of B w.r.t. I, expressed in I (deg/s)
% IC.VelOption        = 'Inertial';                       % Choose in what way you want to define IC velocity - 'Inertial': velocity vector expressed in Inertial frame, 'Body': velocity vector expressed in Body frame, 'WindAngles': express it in terms of absolute velocity and wind angles (Vinf, alfa, beta)
% IC.Vel              = 30;                               % Absolute velocity (m/s)
% IC.alfa             = 0;                                % Angle of attack (deg)
% IC.beta             = 0;                                % Side slip angle (deg)


phi   = IC.orientation(1,1)*pi/180;
theta = IC.orientation(1,2)*pi/180;
psi   = IC.orientation(1,3)*pi/180;

% Building rotation matrix from Body to Inertial frame
c_psi   = cos(psi);   s_psi   = sin(psi);
c_theta = cos(theta); s_theta = sin(theta);
c_phi   = cos(phi);   s_phi   = sin(phi);
  

% Rotation matrix from B to N
b_R_n   = [c_psi*c_theta    (-s_psi*c_phi+s_theta*s_phi*c_psi)      s_psi*s_phi+s_theta*c_psi*c_phi;
           s_psi*c_theta    (c_psi*c_phi +s_psi*s_theta*s_phi)      -s_phi*c_psi+s_psi*s_theta*c_phi;
           -s_theta                  s_phi*c_theta                            c_theta*c_phi          ];

switch IC.VelOption
    case 'Inertial'
        X0(1:3)   = IC.position + r_CG_O;
        X0(4:6)   = (b_R_n'*IC.velocity')' + cross(IC.angular_velocity*pi/180,r_CG_O);
        X0(7:9)   = IC.orientation*pi/180;
        X0(10:12) = IC.angular_velocity*pi/180; 
    case 'Body'
        X0(1:3)   = IC.position + r_CG_O;
        X0(4:6)   = IC.velocity + cross(IC.angular_velocity*pi/180,r_CG_O);
        X0(7:9)   = IC.orientation*pi/180;
        X0(10:12) = IC.angular_velocity*pi/180;
        
    case 'WindAngles'
        alfa = IC.alfa*pi/180;
        beta = IC.beta*pi/180;
        
        Uinf_0    =  IC.Vel*cos(alfa)*cos(beta);
        Vinf_0    = -IC.Vel*sin(beta);
        Winf_0    =  IC.Vel*sin(alfa)*cos(beta);
        
        X0(1:3)   = IC.position + r_CG_O;
        X0(4:6)   = [Uinf_0 Vinf_0 Winf_0] + cross(IC.angular_velocity*pi/180,r_CG_O);
        X0(7:9)   = IC.orientation*pi/180;
        X0(10:12) = IC.angular_velocity*pi/180;
end
        
