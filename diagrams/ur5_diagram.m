zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

% Define robot kinematics parameters
kin.H = [ez -ey -ey -ey -ez -ey];

% BEFORE moving joint origins to be coincident
% kin.P = [zv 0.1625*ez -0.425*ex -0.3922*ex -0.1333*ey -0.0997*ez -0.0996*ey];

% AFTER moving joint origins to be coincident
kin.P = [0.1625*ez zv -0.425*ex -0.3922*ex -0.1333*ey-0.0997*ez zv -0.0996*ey];
kin.joint_type = zeros([6 1]);
R_6T = rot(ex, deg2rad(90));

q_0 = zeros([6 1]);
[R_06, p_0T, p_0x] =  fwdkin_inter(kin, q_0, 1:6);
R_0T = R_06 * R_6T;


diagrams.setup([3 2.5]); hold on
camva(100); % Don't draw out of bounds
[UNIT_SIZE, CYL_HALF_LENGTH] = diagrams.robot_plot(kin, q_0, auto_scale=true, ...
    show_joint_labels=false, ...
    show_base_label=false, ...
    show_task_label=false, ...
    show_arrow_labels = false, ...
    show_base_frame = false, ...
    show_task_frame = false, ...
    show_arrows=false);

diagrams.dot(zv)
diagrams.text(zv, "$\mathcal O_0 $", align=">");
diagrams.text(p_0x(:,1), "$\mathcal O_1 = \mathcal O_2$", align=">", margin=40);
diagrams.text(p_0x(:,3), "$\mathcal O_3$", align="v", margin=15);
diagrams.text(p_0x(:,4), "$\mathcal O_4$", align="v", margin=15);
diagrams.text(p_0x(:,5), "$\mathcal O_5 = \mathcal O_6 $", align="^<", margin=16);
diagrams.text(p_0T, "$\mathcal O_T $", align=">v");

diagrams.text(p_0x(:,1) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,1), "$h_1$");
diagrams.text(p_0x(:,2) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,2), "$h_2$");
diagrams.text(p_0x(:,3) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,3), "$h_3$");
diagrams.text(p_0x(:,4) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,4), "$h_4$", align="v");
diagrams.text(p_0x(:,5) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,5), "$h_5$", align="<");
diagrams.text(p_0x(:,6) - (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,6), "$h_6$", align="^");

diagrams.arrow(p_0x(:,1)+CYL_HALF_LENGTH*kin.H(:,1), p_0x(:,1) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,1));
diagrams.arrow(p_0x(:,2)+CYL_HALF_LENGTH*kin.H(:,2), p_0x(:,2) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,2));
diagrams.arrow(p_0x(:,3)+CYL_HALF_LENGTH*kin.H(:,3), p_0x(:,3) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,3));
diagrams.arrow(p_0x(:,4)+CYL_HALF_LENGTH*kin.H(:,4), p_0x(:,4) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,4));
diagrams.arrow(p_0x(:,5)+CYL_HALF_LENGTH*kin.H(:,5), p_0x(:,5) + (UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,5));
diagrams.arrow(p_0x(:,6)-(UNIT_SIZE+CYL_HALF_LENGTH)*kin.H(:,6), p_0x(:,6)-CYL_HALF_LENGTH*kin.H(:,6));

diagrams.arrow(zv, UNIT_SIZE*ex, color=diagrams.colors.red);
diagrams.arrow(zv, UNIT_SIZE*ey, color=diagrams.colors.green);
diagrams.arrow(zv, UNIT_SIZE*ez, color=diagrams.colors.blue);

diagrams.arrow(p_0T, p_0T+UNIT_SIZE*R_0T(:,1), color=diagrams.colors.red);
diagrams.arrow(p_0T, p_0T+UNIT_SIZE*R_0T(:,2), color=diagrams.colors.green);
diagrams.arrow(p_0T, p_0T+UNIT_SIZE*R_0T(:,3), color=diagrams.colors.blue);

% view(60, 22);
campos([4.0786   -2.6653    2.1623])
camva(7);
camtarget([-0.3820   -0.0899    0.0812]);
diagrams.redraw(); hold off