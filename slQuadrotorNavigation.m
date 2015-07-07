function slQuadrotorNavigation(block)
%% sl_RigidBodyDynamics
% MSFUNTMPL A Template for a MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl' with the name
%   of your S-function.  
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more 
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%  
%   Copyright 2003-2010 The MathWorks, Inc.
%   $Revision: 1.1.6.22 $  
  
%
% The setup method is used to setup the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.  
%   


setup(block);
  
%endfunction

% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
% 
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%

function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 2;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  
  
  block.OutputPort(1).Dimensions       = 9;
    block.OutputPort(2).Dimensions     = 3;

  % Override the output port properties.
  
  for k=1:2
    block.OutputPort(k).SamplingMode = 'sample';
    block.OutputPort(k).DatatypeID  = 0; % double
    block.OutputPort(k).Complexity  = 'Real';
  end
  
    

  % Register the parameters.
  block.NumDialogPrms     = 0;
  %block.DialogPrmsTunable = {'Tunable'};
  %block.DialogPrmsTunable = {'Tunable','Nontunable','SimOnlyTunable'};
  
  
  % Set up the continuous states.
  block.NumContStates = 12;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  block.SampleTimes = [0 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  
  % Specify the block simStateCompliance. The allowed values are:
  %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
  %    'DefaultSimState', < Same SimState as a built-in block
  %    'HasNoSimState',   < No SimState
  %    'CustomSimState',  < Has GetSimState and SetSimState methods
  %    'DisallowSimState' < Errors out when saving or restoring the SimState
  block.SimStateCompliance = 'DefaultSimState';
  
  % -----------------------------------------------------------------
  % The MATLAB S-function uses an internal registry for all
  % block methods. You should register all relevant methods
  % (optional and required) as illustrated below. You may choose
  % any suitable name for the methods and implement these methods
  % as local functions within the same file.
  % -----------------------------------------------------------------
   
  % -----------------------------------------------------------------
  % Register the methods called during update diagram/compilation.
  % -----------------------------------------------------------------
  
  % 
  % CheckParameters:
  %   Functionality    : Called in order to allow validation of the
  %                      block dialog parameters. You are 
  %                      responsible for calling this method
  %                      explicitly at the start of the setup method.
  %   C-Mex counterpart: mdlCheckParameters
  %
  block.RegBlockMethod('CheckParameters', @CheckPrms);

  %
  % SetInputPortSamplingMode:
  %   Functionality    : Check and set input and output port 
  %                      attributes and specify whether the port is operating 
  %                      in sample-based or frame-based mode
  %   C-Mex counterpart: mdlSetInputPortFrameData.
  %   (The DSP System Toolbox is required to set a port as frame-based)
  %
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
  %
  % SetInputPortDimensions:
  %   Functionality    : Check and set the input and optionally the output
  %                      port dimensions.
  %   C-Mex counterpart: mdlSetInputPortDimensionInfo
  %
  block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

  %
  % SetOutputPortDimensions:
  %   Functionality    : Check and set the output and optionally the input
  %                      port dimensions.
  %   C-Mex counterpart: mdlSetOutputPortDimensionInfo
  %
  block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  
  %
  % SetInputPortDatatype:
  %   Functionality    : Check and set the input and optionally the output
  %                      port datatypes.
  %   C-Mex counterpart: mdlSetInputPortDataType
  %
  block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  
  %
  % SetOutputPortDatatype:
  %   Functionality    : Check and set the output and optionally the input
  %                      port datatypes.
  %   C-Mex counterpart: mdlSetOutputPortDataType
  %
  block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  
  %
  % SetInputPortComplexSignal:
  %   Functionality    : Check and set the input and optionally the output
  %                      port complexity attributes.
  %   C-Mex counterpart: mdlSetInputPortComplexSignal
  %
  block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
  
  %
  % SetOutputPortComplexSignal:
  %   Functionality    : Check and set the output and optionally the input
  %                      port complexity attributes.
  %   C-Mex counterpart: mdlSetOutputPortComplexSignal
  %
  block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);
  
  %
  % PostPropagationSetup:
  %   Functionality    : Set up the work areas and the state variables. You can
  %                      also register run-time methods here.
  %   C-Mex counterpart: mdlSetWorkWidths
  %
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  % -----------------------------------------------------------------
  % Register methods called at run-time
  % -----------------------------------------------------------------
  
  % 
  % ProcessParameters:
  %   Functionality    : Call to allow an update of run-time parameters.
  %   C-Mex counterpart: mdlProcessParameters
  %  
  block.RegBlockMethod('ProcessParameters', @ProcessPrms);

  % 
  % InitializeConditions:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlInitializeConditions
  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  % 
  % Start:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlStart
  %
  block.RegBlockMethod('Start', @Start);

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C-Mex counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);

  % 
  % Update:
  %   Functionality    : Call to update the discrete states
  %                      during a simulation step.
  %   C-Mex counterpart: mdlUpdate
  %
  block.RegBlockMethod('Update', @Update);

  % 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C-Mex counterpart: mdlDerivatives
  %
  block.RegBlockMethod('Derivatives', @Derivatives);
    
  % 
  % Projection:
  %   Functionality    : Call to update the projections during a
  %                      simulation step.
  %   C-Mex counterpart: mdlProjections
  %
  block.RegBlockMethod('Projection', @Projection);
  
  % 
  % SimStatusChange:
  %   Functionality    : Call when simulation enters pause mode
  %                      or leaves pause mode.
  %   C-Mex counterpart: mdlSimStatusChange
  %
  block.RegBlockMethod('SimStatusChange', @SimStatusChange);
  
  % 
  % Terminate:
  %   Functionality    : Call at the end of a simulation for cleanup.
  %   C-Mex counterpart: mdlTerminate
  %
  block.RegBlockMethod('Terminate', @Terminate);

  %
  % GetSimState:
  %   Functionality    : Return the SimState of the block.
  %   C-Mex counterpart: mdlGetSimState
  %
  block.RegBlockMethod('GetSimState', @GetSimState);
  
  %
  % SetSimState:
  %   Functionality    : Set the SimState of the block using a given value.
  %   C-Mex counterpart: mdlSetSimState
  %
  block.RegBlockMethod('SetSimState', @SetSimState);

  % -----------------------------------------------------------------
  % Register the methods called during code generation.
  % -----------------------------------------------------------------
  
  %
  % WriteRTW:
  %   Functionality    : Write specific information to model.rtw file.
  %   C-Mex counterpart: mdlRTW
  %
  block.RegBlockMethod('WriteRTW', @WriteRTW);
%endfunction

% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

function CheckPrms(block)
  
%   a = block.DialogPrm(1).Data;
%   if ~strcmp(class(a), 'double')
%     me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter'));
%     throw(me);
%   end
%   
%endfunction

function ProcessPrms(block)

  block.AutoUpdateRuntimePrms;
 
%endfunction

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
%  block.OutputPort(idx).SamplingMode  = fd;
  
%endfunction

function SetInpPortDims(block, idx, di)
  
  block.InputPort(idx).Dimensions = di;
  

%endfunction

function SetOutPortDims(block, idx, di)
  
  
  block.OutputPort(idx).Dimensions = di;
%   if (idx==1)  
%       block.OutputPort(1).Dimensions  = 12;
%   end
%     
  %block.InputPort(1).Dimensions    = di;

%endfunction

function SetInpPortDataType(block, idx, dt)
  
  block.InputPort(idx).DataTypeID = dt;
  %block.OutputPort(1).DataTypeID  = dt;

%endfunction
  
function SetOutPortDataType(block, idx, dt)

  block.OutputPort(idx).DataTypeID  = dt;
  %block.InputPort(1).DataTypeID     = dt;

%endfunction  

function SetInpPortComplexSig(block, idx, c)
  
  block.InputPort(idx).Complexity = c;
%  block.OutputPort(idx).Complexity  = c;

%endfunction 
  
function SetOutPortComplexSig(block, idx, c)

  block.OutputPort(idx).Complexity = c;
  %block.InputPort(1).Complexity    = c;

%endfunction 
    
function DoPostPropSetup(block)
  block.NumDworks = 0;
  %No tenemos estados discretos!!
%   block.Dwork(1).Name            = 'x1';
%   block.Dwork(1).Dimensions      = 12;
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = false;
%   
 
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction

function InitializeConditions(block)

%esto se llama cada vez que se resetea el estado

  global X0;
  block.ContStates.Data=X0;

%endfunction

function Start(block)

%esto se llama una sola vez al comienzo
  global X0;
  block.ContStates.Data=X0;
     
%endfunction

function WriteRTW(block)
  
%    block.WriteRTWParam('matrix', 'M',    [1 2; 3 4]);
%    block.WriteRTWParam('string', 'Mode', 'Auto');
%    
%endfunction

function Outputs(block)
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
  %X=block.ContStates.Data;

  global Quadrotor;
  global Atmosphere;
  global Control_Mode;
  global CtrlParam;
  global t_step;
  persistent t_old;
  
  if isempty(t_old)
      t_old=0;
  end

  X = block.InputPort(1).Data;
  
    [R_est, Omega_est] = Quadrotor_Navigation(CtrlParam, X);
  
  block.OutputPort(1).Data = R_est;             %State
  block.OutputPort(2).Data = Omega_est;
  
  
%endfunction

function Update(block)
  
  
  
%endfunction

function Derivatives(block)

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

%function Xp = RigidBodyDynamics(X, Mass_prop, Net_Forces, Net_Moments_CM)
% Reading STATE VARIABLES

%%PREPARE function parameters



%endfunction

function Projection(block)

states = block.ContStates.Data;
block.ContStates.Data = states+eps; 

%endfunction

function SimStatusChange(block, s)
  
  if s == 0
    disp('Pause in simulation.');
  elseif s == 1
    disp('Resume simulation.');
  end
  
%endfunction
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%endfunction
 
function outSimState = GetSimState(block)

%outSimState = block.Dwork(1).Data;

%endfunction

function SetSimState(block, inSimState)

%block.Dwork(1).Data = inSimState;

%endfunction
