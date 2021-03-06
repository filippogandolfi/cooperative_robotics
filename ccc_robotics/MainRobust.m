function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 40;
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

% UDP Connection with Unity vieuvmswer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('Robust');
% uvms.q
% Initial joint positions. You can change these values to initialize the simulation with a
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.p
% INITIAL POSITION of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [8.5 38.5 -36 0 -0.06 0.5]';

%w rock position saved in uvms
uvms.rock_center = rock_center; % in world frame coordinates

% defines the goal position for the end-effector/tool position task
uvms.toolTargetPosition = [12.2025   37.3748  -39.8860]';
uvms.wRg = rotation(pi, 0, 0);
uvms.wTg = [uvms.wRg uvms.toolTargetPosition; 0 0 0 1];

% defines the target (tg) position for the vehicle position task (1.1)

uvms.targetPosition = [10.5 37.5 -38]';
uvms.wRtg = rotation(0, -0.06, 0.5);
uvms.wTtg = [uvms.wRtg uvms.targetPosition; 0 0 0 1];

% defines the tool control point (end effector - tool)
uvms.eTt = eye(4);

tic
for t = 0:deltat:end_time
    
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);    
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13);
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    [Qp, rhop] = iCAT_task(uvms.A.vNull,    uvms.Jv,    Qp, rhop, uvms.xdot.vNull,  0.0001,   0.01, 10); %Null Vehicle velocity position
    [Qp, rhop] = iCAT_task(uvms.A.jl,    uvms.Jjl,    Qp, rhop, uvms.xdot.jl,  0.0001,   0.01, 10); %Joint Limits
    [Qp, rhop] = iCAT_task(uvms.A.mav,   uvms.Jmav,   Qp, rhop, uvms.xdot.mav, 0.000001, 0.0001, 10); %Minimum Alt. Vehicle
    [Qp, rhop] = iCAT_task(uvms.A.mu,   uvms.Jmu,   Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10); %Manipulability
    [Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10); %Horizontal Attitude
    [Qp, rhop] = iCAT_task(uvms.A.alr,    uvms.Jalr,   Qp, rhop, uvms.xdot.alr, 0.000001, 0.0001, 10); %Alignment rock
    [Qp, rhop] = iCAT_task(uvms.A.l,    uvms.Jl,   Qp, rhop, uvms.xdot.l, 0.000001, 0.0001, 10); %Landing
    [Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10); %Tool
    [Qp, rhop] = iCAT_task(uvms.A.v,    uvms.Jv,    Qp, rhop, uvms.xdot.v,  0.0001,   0.01, 10); %Vehicle position
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10); % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = rhop(8:13);
    
    % Integration
    uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
    
    
    [~, PosErrorLin] = CartError(uvms.wTtg, uvms.wTv);
    [~, ToolErrorLin] = CartError(uvms.vTg , uvms.vTt);
    
    uvms.normVehicleFrameError = norm(PosErrorLin);    
    uvms.normAlignmentError = norm(uvms.misalignment);
    uvms.normToolFrameError = norm(ToolErrorLin);
    
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
    
    
        
    % add debug prints here
    if (mod(t,0.05) == 0)
        debug.t = t
        debug.phase = mission.phase
        debug.dist = uvms.wSensorDistance
        debug.position = [uvms.p(1,1) uvms.p(2,1) uvms.p(3,1)]
        debug.Error = [uvms.normVehicleFrameError uvms.normAlignmentError uvms.normToolFrameError]
        
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    %SlowdownToRealtime(deltat);
    
end
name = uvms.robotname;
fclose(uVehicle);
fclose(uArm);

PrintPlot(plt,name);

end