%% These kinematic parameters are AFTER setting p_12 = p_56 = 0;
kin.P = [0.1625*ez zv -0.425*ex -0.3922*ex -0.1333*ey-0.0997*ez zv -0.0996*ey];
kin.H = [ez -ey -ey -ey -ez -ey];
kin.joint_type = zeros([6 1]);

generate_openrave_xml(kin, "UR5e")

%% These kinematic parameters are BEFORE setting p_12 = p_56 = 0;
kin.P = [zv 0.1625*ez -0.425*ex -0.3922*ex -0.1333*ey -0.0997*ez -0.0996*ey];
kin.H = [ez -ey -ey -ey -ez -ey];
kin.joint_type = zeros([6 1]);

generate_openrave_xml(kin, "UR5e_nonzero")