% IK for UR5 with kinematic parameters hardcoded in
% Used for timing test with MEX
% Almost the same as IK_UR() but with no kin parameter

function [Q, is_LS_vec] = hardcoded_ur5_IK(R_06, p_0T)
% h_2 = h_3 = h_4 and p_12 = p_56 = 0

zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

P = [0.1625*ez zv -0.425*ex -0.3922*ex -0.1333*ey-0.0997*ez zv -0.0996*ey];
H = [ez -ey -ey -ey -ez -ey];
Q = [];
is_LS_vec = [];

p_06 = p_0T - P(:,1) - R_06*P(:,7);

% Find q1 using Subproblem 4
[theta1, theta1_is_ls] = sp_4(...
    H(:,2), p_06, -H(:,1), H(:,2)'*sum(P(:,2:5), 2));

for i_t1 = 1:length(theta1)
    q_1 = theta1(i_t1);
    R_01 = rot(H(:,1), q_1);

    % Find q5 using Subproblem 4
    [theta5, theta5_is_ls] = sp_4(H(:,2),H(:,6),H(:,5), ...
        H(:,2)' * R_01' * R_06 * H(:,6));
    
    for i_t5 = 1:length(theta5)
        q_5 = theta5(i_t5);
        R_45 = rot(H(:,5), q_5);
    
        % solve for R_14 using Subproblem 1
        [theta_14, theta_14_is_LS] = sp_1(...
            R_45*H(:,6),R_01'*R_06*H(:,6), H(:,2));

        % solve for q_6 using Subproblem 1
        [q_6, q_6_is_LS] = sp_1(R_45'*H(:,2), R_06'*R_01*H(:,2), -H(:,6));
    
        % solve for q3 using Subproblem 3
        d_inner = R_01'*p_06-P(:,2) - rot(H(:,2), theta_14)*P(:,5);
        d = norm(d_inner);
        [theta_3, theta_3_is_LS] = sp_3(-P(:,4), P(:,3), H(:,2), d);
    
        for i_q3 = 1:length(theta_3)
            q_3 = theta_3(i_q3);
            
            % solve for q2 using Subproblem 1
            [q_2, q_2_is_LS] = sp_1(...
                P(:,3) + rot(H(:,2), q_3)*P(:,4), d_inner, H(:,2));
    
            % find q4 by subtraction
            q_4 = wrapToPi(theta_14 - q_2 - q_3);
            
            q_i = [q_1; q_2; q_3; q_4; q_5; q_6];
            Q = [Q q_i];
            is_LS_vec = [is_LS_vec theta1_is_ls||theta5_is_ls...
                ||theta_14_is_LS||theta_3_is_LS||q_2_is_LS||q_6_is_LS];
        end
    end
end
end


%% Subproblem solutions

function [theta, is_LS] = sp_1(p1, p2, k)
KxP = cross(k, p1);
A = [KxP -cross(k,KxP)];
x = A'*p2;
theta = atan2(x(1),x(2));

is_LS = abs(norm(p1) - norm(p2)) > 1e-8 ...
    || abs(dot(k,p1) - dot(k,p2)) > 1e-8;
end

function [theta, is_LS] = sp_3(p1, p2, k, d)
[theta, is_LS] = sp_4(p2, p1, k, 1/2 * (dot(p1,p1)+dot(p2,p2)-d^2));
end

function [theta, is_LS] = sp_4(h, p, k, d)
A_11 = cross(k,p);
A_1 = [A_11 -cross(k,A_11)];
A = h'*A_1;
b = d - h'*k*(k'*p);
norm_A_2 = dot(A,A);
x_ls_tilde = A_1'*(h*b);

if norm_A_2 > b^2
    xi = sqrt(norm_A_2-b^2);
    x_N_prime_tilde = [A(2); -A(1)];

    sc_1 = x_ls_tilde + xi*x_N_prime_tilde;
    sc_2 = x_ls_tilde - xi*x_N_prime_tilde;

    theta = [atan2(sc_1(1), sc_1(2)) atan2(sc_2(1), sc_2(2))];
    is_LS = false;
else
    theta = atan2(x_ls_tilde(1), x_ls_tilde(2));
    is_LS = true;
end
end

%% Helper functions
function theta = rand_angle(size)
if nargin < 1
    size = 1;
end
theta = rand(size)*2*pi-pi;
end


function [R, p] = fwdkin(kin, theta)
p = kin.P(:,1);
R = eye(3);

for i = 1:numel(kin.joint_type)
    if (kin.joint_type(i) == 0 || ...       % rotational actuators
                kin.joint_type(i) == 2)        
        R = R*rot(kin.H(:,i),theta(i));
    elseif (kin.joint_type(i) == 1 || ...   % translational actuators
                kin.joint_type(i) == 3)    
        p = p + R*kin.H(:,i)*theta(i);
    end
    p = p + R*kin.P(:,i+1);
end
end

function R=rot(k,theta)
    k = k / norm(k);
    R = eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
end  

function khat = hat(k)
    khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end
  
function a = wrapToPi(a)
    a = mod(a+pi, 2*pi) - pi;
end