function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% example: MAV
% if mav < 5, A = 1;
% if mav > 5.5, A = 0;
% in between, there is a smooth behavior.
uvms.A.mav = uvms.Amiss.mav*DecreasingBellShapedFunction(uvms.d_mav, uvms.dist_limit, 0, 1, uvms.wsensorDistance);

% example: landing
uvms.A.l = uvms.Amiss.l*1;
% example: manipulability
% if mu < 0.02, A = 1;
% if mu > 0.05, A = 0;
% in between, there is a smooth behavior.
uvms.A.mu = uvms.Amiss.mu*DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% phi: misalignment vector between the horizontal plane and the
% longitudinal axis of the vehicle
% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.
uvms.A.ha = uvms.Amiss.ha*IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

% arm tool position control
% always active
uvms.A.t = uvms.Amiss.t*eye(6);

% vehicle position control
% always active
uvms.A.v = uvms.Amiss.v*eye(6);

% Allignment Rock
% always active
uvms.A.alr = uvms.Amiss.alr*eye(6);
