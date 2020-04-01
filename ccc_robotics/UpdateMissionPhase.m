function [uvms, mission] = UpdateMissionPhase(uvms, mission)

[~, PosErrorLin] = CartError(uvms.wTtg, uvms.wTv);

%% ROBUST MISSION
if (strcmp(uvms.robotname,'Robust' ))
    AlgRockError = norm(uvms.misalignment);
    switch mission.phase
        case 1 %safe navigation
            uvms.Amiss.vNull = 0;
            uvms.Amiss.jl = 1;
            uvms.Amiss.mav = IncreasingBellShapedFunction(0.1, 0.15, 0, 1, mission.phase_time);
            uvms.Amiss.mu = 1;
            uvms.Amiss.ha = 1;
            uvms.Amiss.alr = 0;
            uvms.Amiss.l = 0;
            uvms.Amiss.t = 0;
            uvms.Amiss.v = 1;
            if(norm(PosErrorLin) < 0.1)
                mission.phase = 2;
                mission.phase_time = 0;
            end
        case 2 %alignment to the rock
            uvms.Amiss.vNull = 0;
            uvms.Amiss.jl = 1;
            uvms.Amiss.mav = 1;
            uvms.Amiss.mu = 1;
            uvms.Amiss.ha = 1;
            uvms.Amiss.alr = IncreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            uvms.Amiss.l = 0;
            uvms.Amiss.t = 0;
            uvms.Amiss.v = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            if(AlgRockError < 0.07)
                mission.phase = 3;
                mission.phase_time = 0;
            end
        case 3 % landing
            uvms.Amiss.vNull = 0;
            uvms.Amiss.jl = 1;
            uvms.Amiss.mav = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            uvms.Amiss.mu = 1;
            uvms.Amiss.ha = 1;
            uvms.Amiss.alr = 1;
            uvms.Amiss.l = IncreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            uvms.Amiss.t = 0;
            uvms.Amiss.v = 0;
            if(uvms.wSensorDistance < 0.17)
                mission.phase = 4;
                mission.phase_time = 0;
            end
            
        case 4
            uvms.Amiss.vNull = IncreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            uvms.Amiss.mav = 0;
            uvms.Amiss.mu = 1;
            uvms.Amiss.ha = 1;
            uvms.Amiss.alr = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            uvms.Amiss.l = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
            uvms.Amiss.t = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, mission.phase_time);
            uvms.Amiss.v = 0;
            
    end
else 
%% DEXROV MISSION
    if(strcmp(uvms.robotname, 'DexROV'))
        switch mission.phase
            case 1 %safe navigation
                uvms.Amiss.vConstr = 0;
                uvms.Amiss.jl = 1;
                uvms.Amiss.mu = 1;
                uvms.Amiss.ha = 1;
                uvms.Amiss.t = 0;
                uvms.Amiss.v = 1;
                uvms.Amiss.ps = 1;
                if(norm(PosErrorLin) < 0.1)
                    mission.phase = 2;
                    mission.phase_time = 0;
                end
            case 2
                uvms.Amiss.vConstr = IncreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
                uvms.Amiss.jl = 1;
                uvms.Amiss.mu = 1;
                uvms.Amiss.ha = 1;
                uvms.Amiss.t = IncreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
                uvms.Amiss.v = DecreasingBellShapedFunction(0, 0.2, 0, 1, mission.phase_time);
                uvms.Amiss.ps = 1;
        end
        
    end
end
end