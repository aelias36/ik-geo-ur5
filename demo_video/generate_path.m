% Simple linear path between two points
zv = [0;0;0];
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

N = 100;
lambda = linspace(0,1,N);
p_A = [0.4; -0.7; 0.2];
p_B = [0.4; 0.5; 0.2];
p_path = p_A + lambda.*(p_B - p_A);

R_06 = rot(ex, deg2rad(30));

% Compute IK
Q_path = NaN(6, 8, N);
for i = 1:N
    Q_path(:,:,i) = hardcoded_ur5_IK_always_8(R_06, p_path(:,i));
end
plot(squeeze(Q_path(4,:,:))', '.')
ylim([-pi pi])

%% Plot one pose
rob = loadrobot("universalUR5e", DataFormat="column")
%% Initialize plot
q_i = Q_path(:,8,100);
% q_i = [0 0 0 0 0 0]';
q_i(1) = q_i(1)+pi;

figure
set(gcf,'Visible','on', 'Position',[1 1 510 382]);
% show(rob,q_i, Frames="off");
ax = show(rob, q_i, FastUpdate=true, PreservePlot=false, Frames="off");
hold on
axis off
diagrams.utils.plot3_mat([p_A p_B], color=diagrams.colors.red, LineWidth=3)
hold off

light('Position', 2*[1 0 1], 'Color',  0.75*[1 0.9 0.9])
light('Position', 2*[0 1 1], 'Color',  [0.9 0.9 1])
light('Position', 2*[-1 -1 1], 'Color', 0.5*[0.9 0.9 1])
% material([0.2 0.4 0.3 4 1])
material([0.1 0.3 0.3 2 1])
campos([16.1210   15.8068    3.2996])
camva(2.3000)
camtarget([0.1448   -0.1695   0.2])
%% 
for soln_num=1:8
q_path_disp = squeeze(Q_path(:,soln_num,:));
for i = 1:length(q_path_disp)
    q_i = q_path_disp(:,i);
    q_i(1) = q_i(1)+pi;
    ax = show(rob, q_i, FastUpdate=true, PreservePlot=false, Frames="off");
    drawnow
    filename = sprintf('ur5e_%d_%03.0f.png', soln_num, i);

    % pause(1/30)

    saveas(gcf, filename) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp(i + "/" + length(q_path_disp));
end
end

%%
files = dir('ur5e_8_*.png');
files = files(~[files.isdir]);
[~,ix] = sort({files.name}); files = files(ix);

dt = 0.10;                       % 0.10 s per frame = 10 fps
gifout = 'ur5e_anim.gif';

for k = 1:numel(files)
    A = imread(fullfile(files(k).folder, files(k).name));  % RGB
    if k == 1
        [X,map] = rgb2ind(A, 256);                         % build palette
        imwrite(X, map, gifout, 'gif', 'LoopCount', Inf, 'DelayTime', dt);
    else
        X = rgb2ind(A, map);                               % map to first palette
        imwrite(X, map, gifout, 'gif', 'WriteMode', 'append', 'DelayTime', dt);
    end
end