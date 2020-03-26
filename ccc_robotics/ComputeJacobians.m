function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

%% compute manipulability Jacobian
[Jmu_a, uvms.mu] = ComputeManipulability(uvms.bJe, uvms.djdq);
uvms.Jmu = [Jmu_a zeros(1,6)];

%% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%% Vehicle position task
%Jacobian of the arm (in this case we don't need to control the arm so the
%Jacobians is a zero matrix) 
%Jv_a = Jacobian of the Vehicle regarding the Arm end-effector position
uvms.Jv_a  = zeros(6,7);
%Now we initialize the Jv_v = Jacobian of the Vehicle regarding the vehicle
%position)
% NOTE: the jacobian is {ZEROS SOMETHING; SOMETHING ZEROS} because linear 
% and angular velocities are swapped due to the different definitions of 
% the task and control variables
uvms.Jv_v = [zeros(3) uvms.wTv(1:3,1:3); uvms.wTv(1:3,1:3) zeros(3)];
% juxtapose the two Jacobians to obtain the global one
uvms.Jv = [uvms.Jv_a uvms.Jv_v];

%% MAV task
uvms.Jmav = [0 0 0 0 0 1]*uvms.Jv; % it's 0 0 0 0 0 1 and not 0 0 1 0 0 0 because
                                   % the first three componenet are RPY, the other three are XYZ

%% landing task
uvms.Jl = [0 0 0 0 0 1]*uvms.Jv;

%% horizontal attitude Jacobian
kv   = [0 0 1]';
w_kw = [0 0 1]';
v_kw = (uvms.wTv(1:3,1:3))' * w_kw;
uvms.phi = ReducedVersorLemma(v_kw, kv);
if (norm(uvms.phi) > 0)
    nphi = uvms.phi/norm(uvms.phi);
else
    nphi = [0 0 0]';
end
uvms.Jha = [zeros(1,7) nphi'*[zeros(3) eye(3)]];

%% Alignment x axis

%vector joining the vehicle frame to the rock frame wrt to w
w_bVector_vr = uvms.rock_center - uvms.wTv(1:3,4);
% k su vehicle deve andare su world
k = [0 0 1]';
% projection in the inertial horizontal plane (third component = 0)
proj_bVector_vr = (eye(3)-k*k')* w_bVector_vr; %i-kkt
%unit vector joining the vehicle frame to the rock frame
w_uVector_r = proj_bVector_vr / norm(proj_bVector_vr);

%versor i of the vehicle frame
i_vehicle = uvms.wTv(1:3,1:3)*[1 0 0]';

%misaligment (rho) vector with ReducedVectorLemma
uvms.misalignment = ReducedVersorLemma(w_uVector_r, i_vehicle/norm(i_vehicle));
%uvms.Vmisalignment = uvms.wTv(1:3,1:3)*uvms.misalignment;
% Prof hint (we don't want nn)
% theta = norm(uvms.misalignment);
% unit_n = uvms.misalignment/theta;
% Projection matrix on unit_n
% proj_unit_n = unit_n * unit_n';

% w b/a = unit_n*lambda*theta
uvms.Jalr = [zeros(3,7) (-1/norm(w_bVector_vr)^2)*skew(w_bVector_vr)*[1 0 0; 0 1 0;0 0 0]*uvms.wTv(1:3,1:3) eye(3)]; %[3x13] R rotation
%% Joints Limits

uvms.Jjl = [eye(7,7) zeros(7,6)];

%% Manipulator Preferred Shape

uvms.Jps = [eye(4,4) zeros(4,9)];
end