% Create the MEX file
% Set ur5_tutorial as the working directory

% MALAB coder needs an example end effector pose

zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
kin.H = [ez -ey -ey -ey -ez -ey];
kin.P = [0.1625*ez zv -0.425*ex -0.3922*ex -0.1333*ey-0.0997*ez zv -0.0996*ey];
kin.joint_type = zeros([6 1]);
%%
q_example = deg2rad([10 20 30 40 50 60]);
[R_06, p_0T] = fwdkin(kin, q_example);

cfg = coder.config("mex");
cfg.EnableMexProfiling = true;
codegen -config cfg -report hardcoded_ur5_IK.m -args {R_06, p_0T}

%% Test the generated MEX code

q_example_1 = deg2rad([10 20 30 40 50 60]);
[R_06, p_0T] = fwdkin(kin, q_example_1);

Q = hardcoded_ur5_ik_mex(R_06, p_0T);
disp(rad2deg(Q))

q_example_2 = deg2rad([20 30 40 50 60 70]);
[R_06, p_0T] = fwdkin(kin, q_example_2);

Q = hardcoded_ur5_ik_mex(R_06, p_0T);
disp(rad2deg(Q))

%% Generate 10,000 random test cases
N = 10e3;
R_list = NaN(3,3,N);
p_list = NaN(3,N);
for i = 1:N
    q = rand_angle([6 1]);
    [R_06, p_0T] = fwdkin(kin, q);
    R_list(:,:,i) = R_06;
    p_list(:,i) = p_0T;
end

%% Generate mex code for timing test

% Generate fake list with duplicate R, p but the right size
R_list_codegen = NaN(3,3,N);
p_list_codegen = NaN(3,N);
for i = 1:N
    R_list_codegen(:,:,i) = R_list(:,:,1);
    p_list_codegen(:,i) = p_list(:,1);
end

codegen -report ur5_timing_test_inner.m -args {R_list_codegen, p_list_codegen}

%% Conduct timing test
N_outer = 1000;
T_list = NaN([N_outer 1]);
for i = 1:N_outer
 [T_list(i), Q_list] = ur5_timing_test_inner_mex(R_list, p_list);
end
plot(sort(T_list)*1e6, '-x') 
min(T_list)*1e6
 %(microseconds)

%%
for i = 1:10
    Q_i = hardcoded_ur5_ik_mex(R_list(:,:,i), p_list(:,i));
end

%% Accuracy test
N = 10e3;
q_norms = NaN([N 1]);
p_norms = NaN([N 1]);
R_norms = NaN([N 1]);
for i = 1:N
    q = rand_angle([6 1]);
    [R_06, p_0T] = fwdkin(kin, q);
    Q = hardcoded_ur5_ik_mex(R_06, p_0T);
    [q_test, ~, diff_norm] = closest_q(Q, q);
    q_norms(i) = diff_norm;
    [R_06_test, p_0T_test] = fwdkin(kin, q_test);
    p_norms(i) = norm(p_0T - p_0T_test);
    
    % R_norms(i) = real(acos((trace(R_06'*R_06_test) - 1) / 2));
    R = R_06'*R_06_test;                          % relative rotation
    s = 0.5 * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];  % = sinθ * u
    c = 0.5 * (trace(R) - 1);              % = cosθ
    sn = norm(s);
    
    % Primary formula
    R_norms(i) = atan2(sn, c);
end

%% Error in q
semilogy(sort(q_norms), linewidth=2);
title("Joint-space Error")
xlabel("Solution # (Sorted)")
ylabel("||\Delta q|| (Rad)")

%% Error in p_0T
semilogy(sort(p_norms), linewidth=2);
title("Task-Space Error (Position)")
xlabel("Solution # (Sorted)")
ylabel("||\Delta p_{0T}|| (m)")

%% Error in R_06
semilogy(sort(R_norms), linewidth=2);
title("Task-Space Error (Rotation)")
xlabel("Solution # (Sorted)")
ylabel("\angle \Delta R_{06} (rad)")