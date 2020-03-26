function MainDexrov
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 45;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

%w rock position
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates


% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('DexROV');
% uvms.q
% Initial joint positions. You can change these values to initialize the simulation with a
% different starting position for the arm
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';

%w rock position saved in uvms
uvms.rock_center = rock_center; % in world frame coordinates


% initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.3;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the target (tg) position for the vehicle position task (1.1)

uvms.targetPosition = [-1.94 10.66 -29.5]';
uvms.wRtg = rotation(0,0,-pi/2);
uvms.wTtg = [uvms.wRtg uvms.targetPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
%% TPIK 1!
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13);
    % the sequence of iCAT_task calls defines the priority
    [Qp, rhop] = iCAT_task(uvms.A.jl,    uvms.Jjl,    Qp, rhop, uvms.xdot.jl,  0.0001,   0.01, 10); %Joint Limits
    [Qp, rhop] = iCAT_task(uvms.A.mu,   uvms.Jmu,   Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10); %Manipulability
    [Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10); %Horizontal Attitude
    [Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10); %Tool
    [Qp, rhop] = iCAT_task(uvms.A.v,    uvms.Jv,    Qp, rhop, uvms.xdot.v,  0.0001,   0.01, 10); %Vehicle position
    [Qp, rhop] = iCAT_task(uvms.A.ps,    uvms.Jps,    Qp, rhop, uvms.xdot.ps,  0.0001,   0.01, 10); % Preferred Shape
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
%% TPIK 2!
    rhop2 = zeros(13,1);
    Qp2 = eye(13); 
    
    [Qp2, rhop2] = iCAT_task(uvms.A.vConstr,    uvms.JvConstr,    Qp2, rhop2, uvms.xdot.vConstr,  0.0001,   0.01, 10); %Vehicle constrained velocity position
    [Qp2, rhop2] = iCAT_task(uvms.A.jl,    uvms.Jjl,    Qp2, rhop2, uvms.xdot.jl,  0.0001,   0.01, 10); %Joint Limits
    [Qp2, rhop2] = iCAT_task(uvms.A.mu,   uvms.Jmu,   Qp2, rhop2, uvms.xdot.mu, 0.000001, 0.0001, 10); %Manipulability
    [Qp2, rhop2] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp2, rhop2, uvms.xdot.ha, 0.0001,   0.01, 10); %Horizontal Attitude
    [Qp2, rhop2] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp2, rhop2, uvms.xdot.t,  0.0001,   0.01, 10); %Tool
    [Qp2, rhop2] = iCAT_task(uvms.A.v,    uvms.Jv,    Qp2, rhop2, uvms.xdot.v,  0.0001,   0.01, 10); %Vehicle position
    [Qp2, rhop2] = iCAT_task(uvms.A.ps,    uvms.Jps,    Qp2, rhop2, uvms.xdot.ps,  0.0001,   0.01, 10); % Preferred Shape
    [Qp2, rhop2] = iCAT_task(eye(13),     eye(13),    Qp2, rhop2, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one        
    
%%
    % updating the feedback for TPIK 2 with the p_dot for next loop
    uvms.p_dot_feedback = uvms.p_dot;
    % get only the variable of velocity for integration
    uvms.p_dot = rhop(8:13);
    % get the two variables for integration
    uvms.q_dot = rhop2(1:7);
    
    % Integration
    uvms.q = uvms.q + uvms.q_dot*deltat;
    % sinusoidal disturbance
    uvms.disturbance = (0.15*[0 0 sin(t)]');
    
    % beware: p_dot should be projected on <v>
    uvms.p_dot(1:3,1) = uvms.p_dot(1:3,1) + uvms.vTw(1:3, 1:3) * uvms.disturbance;
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
    
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
    
    [angError, linError] = CartError(uvms.wTg , uvms.wTt);
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        debug.t = t
        debug.phase = mission.phase
        debug.Error = [round(angError',3) round(linError',3)]
        debug.p = uvms.p'
        debug.q = uvms.q'
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end