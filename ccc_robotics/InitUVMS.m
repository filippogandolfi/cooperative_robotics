function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix between the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.robotname = robotname;

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.q_ddot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';
uvms.p_ddot = [0 0 0 0 0 0]';
uvms.p_dot_feedback = [0 0 0 0 0 0]';
%rock center
uvms.rock_center = [];

%misaligment
uvms.misalignment = [];
uvms.nmisalignment = [];

% joint limits corresponding to the actual MARIS arm configuration

uvms.jlmin  = [-2.9; -1.6; -2.9; -2.95; -2.9; -1.65; -2.8]; 
             %[-166�  -91� -166�  -169� -166�   -95� -161�]
uvms.jlmax  = [ 2.9;  1.65; 2.9;  0.01;  2.9; 1.25; 2.8]; 
             %[ 166�    95� 166�     1�  166�   72�  161�]

uvms.jl_guard = zeros(7,1); % safe guard for joint limits in order to avoid to reach extrema
uvms.jl_active_min = zeros(7,1); %upper guard
uvms.jl_active_max = zeros(7,1); %lower guard
uvms.jl_mid = zeros(7,1); %mid point between the upper and the lower guards

%Preferred manipulator shape

uvms.prefShape  = [-0.0031; 1.2586; 0.0128; -1.2460]; 


for i = 1:7
    
    uvms.jl_guard(i) = ( abs(uvms.jlmin(i)) + abs(uvms.jlmax(i)) ) / 10;
    uvms.jl_active_min(i) = uvms.jlmin(i) + uvms.jl_guard(i);
    uvms.jl_active_max(i) = uvms.jlmax(i) - uvms.jl_guard(i);
    
    uvms.jl_mid(i) = ( abs(uvms.jlmin(i)) + abs(uvms.jlmax(i)) ) / 2;
end

% MAV task distance limits
delta = 0.50;
uvms.d_mav = 1;
uvms.dist_limit = uvms.d_mav+delta;

% landing task distance limits
delta_landing = 0.15;
d = 0;
uvms.dist_floor = d+delta_landing;
% to be compute at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.vTtg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.psi = zeros(3,1);
uvms.xi = zeros(3,1);
uvms.virtualFrameVelocity = zeros(6,1);
uvms.sensorDistance = 0;
uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jcc = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jc = [];
uvms.Jca = [];
uvms.Jmav = [];
uvms.Jl = [];
uvms.Jalr = [];
uvms.Jps = [];

uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.mav = [];
uvms.xdot.cc = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
uvms.xdot.v = [];
uvms.xdot.c = [];
uvms.xdot.ca = [];
uvms.xdot.l = [];
uvms.xdot.alr = [];
uvms.xdot.ps = [];
uvms.xdot.vConstr = [];

uvms.A.jl = zeros(7,7);
uvms.A.mu = zeros(1,1);
uvms.A.cc = zeros(1,1);
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
uvms.A.v = zeros(6,6);
uvms.A.c = [];
uvms.A.ca = zeros(3,3);
uvms.A.mav = zeros(1,1);
uvms.A.l = zeros(1,1);
uvms.A.alr = zeros(1,1);
uvms.A.vNull = zeros(6,6);
uvms.A.ps =  zeros(4,4);
uvms.A.vConstr = zeros(6,6);

uvms.Amiss.jl = 0;
uvms.Amiss.mu = 0;
uvms.Amiss.cc = 0;
uvms.Amiss.ha = 0;
uvms.Amiss.t = 0;
uvms.Amiss.v = 0;
uvms.Amiss.c = 0;
uvms.Amiss.ca = 0;
uvms.Amiss.mav = 0;
uvms.Amiss.l = 0;
uvms.Amiss.alr= 0;
uvms.Amiss.vNull= 0;
uvms.Amiss.ps = 0;
uvms.Amiss.vConstr= 0;

uvms.toolFrameError = zeros(6,1);
uvms.normVehicleFrameError = zeros(3,1);    
uvms.normAlignmentError = zeros(3,1); 
uvms.normToolFrameError = zeros(3,1);

end