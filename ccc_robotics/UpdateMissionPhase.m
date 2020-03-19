function [uvms, mission] = UpdateMissionPhase(uvms, mission)

[~, PosErrorLin] = CartError(uvms.wTtg, uvms.wTv);
AlgRockError = norm(uvms.misalignment);

% example: manipulability
% if mu < 0.02, A = 1;
% if mu > 0.05, A = 0;
% in between, there is a smooth behavior with DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior with IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));


switch mission.phase
    case 1 %safe navigation
        % mav, mu, ha, v
        if(norm(PosErrorLin) <= 0.45)
            uvms.startGoDown = uvms.wsensorDistance;
            mission.phase = 2;
        end
        uvms.Amiss.mu = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.t = 0;
        uvms.Amiss.v = IncreasingBellShapedFunction(0, 0.5, 0, 1, norm(PosErrorLin));
        uvms.Amiss.mav = IncreasingBellShapedFunction(0, 0.5, 0, 1, norm(PosErrorLin));
        %uvms.Amiss.l = DecreasingBellShapedFunction(0.5, 1, 0, 1, norm(PosErrorLin));
        uvms.Amiss.l=0;
        uvms.Amiss.alr=0;
    case 2
        if(AlgRockError<= 0.3)
            uvms.startGoDown = uvms.wsensorDistance;
            mission.phase = 3;
        end
        uvms.Amiss.mu = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.t = 0;
        uvms.Amiss.v = 0;
        uvms.Amiss.mav = IncreasingBellShapedFunction(0, 0.5, 0, 1, norm(PosErrorLin));
        uvms.Amiss.l = 0;
%         uvms.Amiss.alr = IncreasingBellShapedFunction(0, 0.5, 0, 1, AlgRockError);
        uvms.Amiss.alr = 1;
    case 3 % landing with no arm
        uvms.Amiss.mu = 1;
        uvms.Amiss.ha = 1;
        uvms.Amiss.t = 0;
        uvms.Amiss.l = DecreasingBellShapedFunction(uvms.startGoDown-0.5, uvms.startGoDown+0.5, 0, 1, uvms.wsensorDistance);
        uvms.Amiss.v = 0;
        uvms.Amiss.mav = 0;
%         if(norm(PosErrorLin) > 0.5)
%             mission.phase = 1;
%         end
        
end


% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.