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

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.q_ddot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';
uvms.p_ddot = [0 0 0 0 0 0]';

%rock center
uvms.rock_center = [];

%misaligment
uvms.misalignment = [];

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% MAV task distance limits
delta_mav = 0.50;
uvms.d_mav = 1;
uvms.dist_limit = uvms.d_mav+delta_mav;

% landing task distance limits
delta_landing = 0.15;
d = 0;
uvms.dist_floor = d+delta_landing;
uvms.startGoDown = 0;
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
    
uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.cc = zeros(1,1);
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
uvms.A.v = zeros(6,6);
uvms.A.c = [];
uvms.A.ca = zeros(3,3);
uvms.A.mav = zeros(1,1);
uvms.A.l = 1;
uvms.A.alr = zeros(1,1);

uvms.Amiss.jl = 1;
uvms.Amiss.mu = 1;
uvms.Amiss.cc = 1;
uvms.Amiss.ha = 1;
uvms.Amiss.t = 1;
uvms.Amiss.v = 1;
uvms.Amiss.c = 1;
uvms.Amiss.ca = 1;
uvms.Amiss.mav = 1;
uvms.Amiss.l = 1;
uvms.Amiss.alr=0;


uvms.toolFrameError = zeros(6,1);
uvms.totalError = zeros(6,1);
end