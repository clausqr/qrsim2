%% RigidBodyDynamics - Differential equations of 3D motion of a rigid body

% Author: Roberto A. Bunge, Ph.D. Candidate 
% Department of Aeronautics and Astronautics
% Stanford University
% email address: rbunge@stanford.edu 
% June 2011; Last revision: 18-Sep-2011

% Given the mass properties and the net force and moment about the
% center of mass, returns the derivative of the state
% vector of the rigid body.
%
% Notation:
%    I  = inertial frame
%    B  = body fixed frame.  Origin of B doesn't necessarily coincide with CM
%    CM = center of mass 
%    p  = the suffix "p", indicates the derivative of the variable (e.g.
%    "wxp" means derivative of wx, which is angular acceleration in x)
%
% Inputs:
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
%                     
%    Mass_prop - structure that contains mass and inertia scalars
%
%    Net_Forces - total force acting on rigid body, expressed in B    
%
%    Net_Moments_CM - total moment acting on rigid body about center of mass, expressed in B.
%
% Outputs:
%    Xp - first derivative of state vector X
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none

function Xp = RigidBodyDynamics(X, Mass_prop, Net_Forces, Net_Moments_CM)
% Reading STATE VARIABLES
u     = X(4,1);             
v     = X(5,1);             
w     = X(6,1);             
phi   = X(7,1);             
theta = X(8,1);             
psi   = X(9,1);             
wx    = X(10,1);            
wy    = X(11,1);            
wz    = X(12,1);            

% Reading Mass properties.  Inertias are about CM
mass = Mass_prop.Mass;            % kg

Ixx = Mass_prop.Ixx;              % kg.m^2
Iyy = Mass_prop.Iyy;              % kg.m^2
Izz = Mass_prop.Izz;              % kg.m^2
Ixy = Mass_prop.Ixy;              % kg.m^2
Ixz = Mass_prop.Ixz;              % kg.m^2
Iyz = Mass_prop.Iyz;              % kg.m^2

% Applied force and torque
Fx = Net_Forces(1);
Fy = Net_Forces(2);
Fz = Net_Forces(3);
Tx = Net_Moments_CM(1);
Ty = Net_Moments_CM(2);
Tz = Net_Moments_CM(3);

% Differential equations
m = mass;
up = Fx/m + v*wz - w*wy;
vp = Fy/m + w*wx - u*wz;
wp = Fz/m + u*wy - v*wx;

wxp = ((Ixy*Izz-Ixz*Iyz)*(Ty+wx*(Ixz*wx+Iyz*wy+Izz*wz)-wz*(Ixx*wx+Ixy*wy+Ixz*wz))+(Ixy*Iyz-Ixz*Iyy)*(wx*(Ixy*wx+Iyy*wy+Iyz*wz)-Tz-wy*(Ixx*wx+Ixy*wy+Ixz*wz))+(Iyy*Izz-Iyz^2)*(wy*(Ixz*wx+Iyz*wy+Izz*wz)-Tx-wz*(Ixy*wx+Iyy*wy+Iyz*wz)))/(  ...
Ixx*Iyz^2-Izz*(Ixx*Iyy-Ixy^2)-Ixz*(2*Ixy*Iyz-Ixz*Iyy));

wyp = -((Ixx*Izz-Ixz^2)*(Ty+wx*(Ixz*wx+Iyz*wy+Izz*wz)-wz*(Ixx*wx+Ixy*wy+Ixz*wz))+(Ixx*Iyz-Ixy*Ixz)*(wx*(Ixy*wx+Iyy*wy+Iyz*wz)-Tz-wy*(Ixx*wx+Ixy*wy+Ixz*wz))+(Ixy*Izz-Ixz*Iyz)*(wy*(Ixz*wx+Iyz*wy+Izz*wz)-Tx-wz*(Ixy*wx+Iyy*wy+Iyz*wz)))/(  ...
Ixx*Iyz^2-Izz*(Ixx*Iyy-Ixy^2)-Ixz*(2*Ixy*Iyz-Ixz*Iyy));

wzp = ((Ixx*Iyz-Ixy*Ixz)*(Ty+wx*(Ixz*wx+Iyz*wy+Izz*wz)-wz*(Ixx*wx+Ixy*wy+Ixz*wz))+(Ixx*Iyy-Ixy^2)*(wx*(Ixy*wx+Iyy*wy+Iyz*wz)-Tz-wy*(Ixx*wx+Ixy*wy+Ixz*wz))+(Ixy*Iyz-Ixz*Iyy)*(wy*(Ixz*wx+Iyz*wy+Izz*wz)-Tx-wz*(Ixy*wx+Iyy*wy+Iyz*wz)))/(  ...
Ixx*Iyz^2-Izz*(Ixx*Iyy-Ixy^2)-Ixz*(2*Ixy*Iyz-Ixz*Iyy));

% Converting from wx, wy, wz, to psi_p, theta_p, phi_p
c_psi   = cos(psi);   s_psi   = sin(psi);
c_theta = cos(theta); s_theta = sin(theta);
c_phi   = cos(phi);   s_phi   = sin(phi);
A       = (1/c_theta)*[c_theta s_theta*s_phi s_theta*c_phi;
                       0       c_theta*c_phi -c_theta*s_phi;
                       0       s_phi         c_phi];
vect    = A*[wx wy wz]';
phi_p   = vect(1,1);
theta_p = vect(2,1);
psi_p   = vect(3,1);     

% Rotation matrix from B to I
B_R_I   = [c_psi*c_theta    (-s_psi*c_phi+s_theta*s_phi*c_psi)      s_psi*s_phi+s_theta*c_psi*c_phi;
           s_psi*c_theta    (c_psi*c_phi +s_psi*s_theta*s_phi)      -s_phi*c_psi+s_psi*s_theta*c_phi;
           -s_theta                  s_phi*c_theta                            c_theta*c_phi          ];
       
        %     [          cy*cz,          cy*sz,            -sy]
        %     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
        %     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]

Xp(1:3,1) = B_R_I*[u  v  w]';
Xp(4,1)   = up;
Xp(5,1)   = vp;
Xp(6,1)   = wp;
Xp(7,1)   = phi_p;
Xp(8,1)   = theta_p;
Xp(9,1)   = psi_p;
Xp(10,1)  = wxp;
Xp(11,1)  = wyp;
Xp(12,1)  = wzp;



