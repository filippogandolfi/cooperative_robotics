function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here
% vehicle

% reference for manipulability
uvms.xdot.mu = 0.1 * (0.12 - uvms.mu);

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);
% reference for vehicle position control task
[ang, lin] = CartError(uvms.wTtg, uvms.wTv);
uvms.xdot.v = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.v(1:3) = Saturate(uvms.xdot.v(1:3), 0.2);
uvms.xdot.v(4:6) = Saturate(uvms.xdot.v(4:6), 0.2);
uvms.xdot.v;

% reference for MAV control task
k=[0; 0; 1];
uvms.wsensorDistance = k'*uvms.wTv(1:3,1:3)*[0 0 uvms.sensorDistance]';
uvms.xdot.mav = 0.2*(uvms.dist_limit - uvms.wsensorDistance);
% limit the requested velocities...
% uvms.xdot.mav(1:3) = Saturate(uvms.xdot.mav(1:3), 0.2);
% uvms.xdot.mav(4:6) = Saturate(uvms.xdot.mav(4:6), 0.2);
uvms.xdot.mav;

% reference for Landing control task
uvms.xdot.l = 0.2*(uvms.dist_floor - uvms.wsensorDistance);
% limit the requested velocities...
% SATURIAMO!!!!!
% uvms.xdot.mav(1:3) = Saturate(uvms.xdot.mav(1:3), 0.2);
% uvms.xdot.mav(4:6) = Saturate(uvms.xdot.mav(4:6), 0.2);
uvms.xdot.l;

% reference for horizontal attitude
uvms.xdot.ha = -0.1 * norm(uvms.phi); 

% reference for alignment task 
uvms.xdot.alr = -0.3*(uvms.misalignment); %TO-DO