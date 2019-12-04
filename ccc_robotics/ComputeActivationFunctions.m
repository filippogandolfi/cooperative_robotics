function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% example: MAV
% if mav < -1, A = 1;
% if mav > -0.25, A = 0;
% in between, there is a smooth behavior.
uvms.A.mav = IncreasingBellShapedFunction(-1, -0.25, 0, 1, uvms.xdot.mav);

% example: manipulability
% if mu < 0.02, A = 1;
% if mu > 0.05, A = 0;
% in between, there is a smooth behavior.
uvms.A.mu = DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% phi: misalignment vector between the horizontal plane and the
% longitudinal axis of the vehicle
% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.
uvms.A.ha = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

% arm tool position control
% always active
uvms.A.t = eye(6);

% vehicle position control
% always active
uvms.A.v = eye(6);
