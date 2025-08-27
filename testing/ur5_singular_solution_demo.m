% Generate singular pose

zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

kin.P = [0.1625*ez zv -0.425*ex -0.3922*ex -0.1333*ey-0.0997*ez zv -0.0996*ey];
kin.H = [ez -ey -ey -ey -ez -ey];
kin.joint_type = zeros([1 6]);

q_0 = zeros([6 1]);
[R_06, p_0T] = fwdkin(kin, q_0);

%% Confirm singularity
J = robotjacobian(kin, q_0);
disp("Min singular value: " + min(svd(J)))

%% IK still works at singularity
[Q, is_LS] = hardcoded_ur5_IK(R_06, p_0T)