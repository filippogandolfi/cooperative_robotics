function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
%NOTE:
%DECREASING
% This function is defined as follows
% ymax, if x < xmin
% ymin, if x > xmax
%INCREASING
% This function is defined as follows
% ymin, if x < xmin
% ymax, if x > xmax
%% MINIMUM ALTITUDE VEHICLE

% WITH MISSION PHASES
uvms.A.mav = uvms.Amiss.mav*DecreasingBellShapedFunction(uvms.d_mav, uvms.dist_limit, 0, 1, uvms.wSensorDistance);

% WITHOUT MISSION PHASES
%uvms.A.mav = DecreasingBellShapedFunction(uvms.d_mav, uvms.dist_limit, 0, 1, uvms.wSensorDistance);
%% LANDING
% WITH MISSION PHASES
 uvms.A.l = uvms.Amiss.l*1;

% WITHOUT MISSION PHASES
%uvms.A.l = uvms.A.l*1;
%% MANIPULABILITY
% if mu > 0.05, A = 0;
% if mu < 0.02, A = 1;
% in between, there is a smooth behavior.

% WITH MISSION PHASES
uvms.A.mu = uvms.Amiss.mu*DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% WITHOUT MISSION PHASES
%uvms.A.mu = DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

%% HORIZONTAL ATTITUDE
% phi: misalignment vector between the horizontal plane and the
% longitudinal axis of the vehicle
% if norm(phi) > 0.1, A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.

% WITH MISSION PHASES
 uvms.A.ha = uvms.Amiss.ha*IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

% WITHOUT MISSION PHASES
%uvms.A.ha = IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

%% ARM TOOL POSITION CONTROL
% always active

% WITH MISSION PHASES
uvms.A.t = uvms.Amiss.t*eye(6);

% WITHOUT MISSION PHASES
%uvms.A.t = eye(6);

%% VEHICLE POSITION CONTROL
% always active

% WITH MISSION PHASES
uvms.A.v = uvms.Amiss.v*eye(6);

% WITHOUT MISSION PHASES
%uvms.A.v = eye(6);

%% ALLIGNMENT ROCK
% always active

% WITH MISSION PHASES
uvms.A.alr = uvms.Amiss.alr*eye(3);
% WITHOUT MISSION PHASES
%uvms.A.alr = eye(3);

