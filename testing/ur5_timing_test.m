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
rng("default") % Reset random number generator
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
N = 1000;
R_list_codegen = NaN(3,3,N);
p_list_codegen = NaN(3,N);
for i = 1:N
    R_list_codegen(:,:,i) = R_list(:,:,1);
    p_list_codegen(:,i) = p_list(:,1);
end

codegen -report ur5_timing_test_inner.m -args {R_list_codegen, p_list_codegen}


%% Conduct timing test
N_outer = 1000;
N = 1000;
T_list = NaN([N_outer 1]);
for i = 1:N_outer
 [T_list(i), Q_list] = ur5_timing_test_inner_mex(R_list(:,:,1:N), p_list(:,1:N));
end
plot(sort(T_list)*1e6, '-x') 
min(T_list)*1e6 % microseconds

%%
for i = 1:10
    % Q_i = hardcoded_ur5_ik_mex(R_list(:,:,i), p_list(:,i));
    Q_i = hardcoded_ur5_IK_one_solution(R_list(:,:,i), p_list(:,i));
end

%% Accuracy test
rng("default")
N = 10e3;
% N = 1000;
q_norms = NaN([N 1]);
p_norms = NaN([N 1]);
R_norms = NaN([N 1]);
for i = 1:N
    q = rand_angle([6 1]);
    [R_06, p_0T] = fwdkin(kin, q);
    Q = hardcoded_ur5_IK_mex(R_06, p_0T);
    % Q = reshape(Q_list(:,i), 6, 8);
    [q_test, ~, diff_norm] = closest_q(Q, q);
    q_norms(i) = diff_norm;
    [R_06_test, p_0T_test] = fwdkin(kin, q_test);
    p_norms(i) = norm(p_0T - p_0T_test);
    
    % R_norms(i) = real(acos((trace(R_06'*R_06_test) - 1) / 2));
    R = R_06'*R_06_test;
    s = 0.5 * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    c = 0.5 * (trace(R) - 1);
    sn = norm(s);
    
    % Primary formula
    R_norms(i) = atan2(sn, c);
end

%% Error medians
clc
median(q_norms)
median(R_norms)
median(p_norms)


%% Error in q
% Export with 600 dpi

semilogy(sort(q_norms), '.k');
% title("Joint-space Error")
xlabel("Solution \# (Sorted)", Interpreter="latex")
ylabel("$|\!|\Delta q|\!|$ (rad)", Interpreter="latex")

fontsize(12, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%% Error in R_06
semilogy(sort(R_norms), '.k');
% title("Task-Space Error (Rotation)")
xlabel("Solution \# (Sorted)", Interpreter="latex")
ylabel("$\angle(\Delta R_{06})$ (rad)", Interpreter="latex")

fontsize(12, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%% Error in p_0T
semilogy(sort(p_norms), '.k');
% title("Task-Space Error (Position)")
xlabel("Solution \# (Sorted)", Interpreter="latex")
ylabel("$|\!|\Delta p_{0T}|\!|$ (m)", Interpreter="latex")

fontsize(12, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

%% Show all 3 on one graph
semilogy(sort(q_norms), '.k'); hold on
semilogy(sort(R_norms), '.', color=diagrams.colors.red);
semilogy(sort(p_norms), '.', color=diagrams.colors.blue);
 hold off
% title("Joint-space Error")
xlabel("Solution \# (Individually sorted)", Interpreter="latex")
ylabel("Solution Error", Interpreter="latex")

fontsize(12, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

[~, objh] = legend(["$|\!|\Delta q|\!|$ (rad)", "$\angle(\Delta R_{06})$ (rad)", "$|\!|\Delta p_{0T}|\!|$ (m)"], ...
    Interpreter="latex", Location="northwest");
objhl = findobj(objh, 'type', 'line');
set(objhl, 'LineStyle', '-', 'Marker','none', 'LineWidth',2);
%% Test angle calculation is correct

R = rot(rand_normal_vec, 2.123e-8);
s = 0.5 * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
c = 0.5 * (trace(R) - 1);
sn = norm(s);

atan2(sn, c) % atan2 is accurate
real(acos((trace(R) - 1) / 2)) % acos is NOT accurate
