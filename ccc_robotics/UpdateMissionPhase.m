function [uvms, mission] = UpdateMissionPhase(uvms, mission)

[~, PosErrorLin] = CartError(uvms.wTtg, uvms.wTv);
AlgRockError = norm(uvms.misalignment);

%NOTE:
%DECREASING
% This function is defined as follows
% ymax, if x < xmin
% ymin, if x > xmax
%INCREASING
% This function is defined as follows
% ymin, if x < xmin
% ymax, if x > xmax

switch mission.phase
    case 1 %safe navigation
        % mav, mu, ha, v
        if(norm(PosErrorLin) <= 0.1)
            mission.phase = 2;
            mission.phase_time = 0;
        end
        
        uvms.Amiss.vNull = 0;
        uvms.Amiss.mav = 1;
        uvms.Amiss.mu = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.alr = 0;
        uvms.Amiss.l = 0;
        uvms.Amiss.t = 0;
        uvms.Amiss.v = 1;
        
    case 2 %alignment to the rock
        if(AlgRockError<= 0.1)
            uvms.startGoDown = uvms.wSensorDistance;
            mission.phase = 3;
        end
        
        uvms.Amiss.mav = 1;
        uvms.Amiss.mu = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.t = 0;
        uvms.Amiss.v = 0;
        uvms.Amiss.l = 0;
        uvms.Amiss.alr = DecreasingBellShapedFunction(0.1, 1, 0, 1, AlgRockError);
        uvms.Amiss.vNull = 0;
        
    case 3 % landing
        if(uvms.wSensorDistance < 0.17)
            mission.phase = 4;
            mission.phase_time = 0;
        end
        uvms.Amiss.mu = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.l = DecreasingBellShapedFunction(uvms.startGoDown-0.5, uvms.startGoDown+0.5, 0, 1, uvms.wSensorDistance);
        uvms.Amiss.alr = 1;
        uvms.Amiss.t = 0;
        uvms.Amiss.v = 0;
        uvms.Amiss.mav = 0;
        uvms.Amiss.vNull = 0;
       
    case 4 
        uvms.Amiss.vNull = 1;
        uvms.Amiss.mu = 1;
        uvms.Amiss.t = IncreasingBellShapedFunction(0, 0.15, 0, 1, mission.phase_time);
        uvms.Amiss.l = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.alr = 0;
        uvms.Amiss.v = 0;
        uvms.Amiss.mav = 0;
        
end


% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.