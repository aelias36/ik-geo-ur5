%% This function gets converted to MEX code to conduct the timing test

function [T, Q_list] = ur5_timing_test_inner(R_list, p_list)
N = 1000;
Q_list = NaN(6*8, N);

% Q_list = NaN(6, N);

% Timer starts here
tic;

for i = 1:N
    R_i = R_list(:,:,i);
    p_i = p_list(:,i);
    Q_i = hardcoded_ur5_IK(R_i, p_i);
    Q_list(1:numel(Q_i),i) = Q_i(:);

    % Q_list(:,i) = hardcoded_ur5_IK_one_solution(R_i, p_i);

end
% Timer ends here
T_total = toc;
T = T_total/N;
end