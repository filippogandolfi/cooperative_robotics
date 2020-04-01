function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;
plt.q_ddot(:, loop) = uvms.q_ddot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;
plt.p_ddot(:, loop) = uvms.p_ddot;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl);
plt.a(8, loop) = uvms.A.mu;
plt.a(9, loop) = uvms.A.ha(1,1);
plt.a(10, loop) = uvms.A.mav;
plt.a(11, loop) = uvms.A.l;
plt.a(12, loop) = uvms.A.alr(1,1);
plt.a(13, loop) = uvms.A.vNull(1,1);
plt.a(14, loop) = uvms.A.t(1,1);
plt.a(15, loop) = uvms.A.v(1,1);

plt.Error(1,loop) = uvms.normVehicleFrameError ;
plt.Error(2,loop) = uvms.normAlignmentError;
plt.Error(3,loop) = uvms.normToolFrameError;


plt.toolFrameError(:, loop) = uvms.toolFrameError;

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

%joint limit
plt.jlmin(:,loop) = uvms.jlmin;
plt.jlmax(:,loop) = uvms.jlmax;
end