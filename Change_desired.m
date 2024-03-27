你要将下面代码放在GitHub上，写一个readme，大概说明这是一个基于casadi的简易优化框架，考虑运动学或是什么优化载有机械臂的四足机器人达到期望轨迹的过程

import casadi.*


n_steps = 100;
n_joints = 5;
circle_radius = 5;

% Desired circular trajectory
theta = linspace(0, 2*pi, n_steps);
desired_trajectory = [15 + circle_radius * cos(theta)',  10 + circle_radius * sin(theta)',  15*ones(n_steps, 1)];

% Decision variables
joint_angles_opt = MX.sym('joint_angles_opt', n_steps, n_joints);
base_pos_opt = MX.sym('base_pos_opt', n_steps, 3);

% --- 目标函数 ---
J = 0;
for i = 1:n_steps
    % 计算机械臂末端的位置
    [~, ~, end_effector_pos] = compute_arm_positions(base_pos_opt(i, :)+ [0, 0, 2/2], joint_angles_opt(i, :), 5, 4, 2);
    
    % 计算机械臂末端与期望轨迹之间的距离
    distance = norm(end_effector_pos - desired_trajectory(i, :));
    J = J + distance^2;
end

% 设置决策变量的初始猜测和边界
joint_angles_init = zeros(n_steps, n_joints);
base_pos_init = repmat([10, 5, 7.5], n_steps, 1);
initial_guess = [joint_angles_init(:); base_pos_init(:)];
lower_bounds = [repmat([-180; -90; -90; -90; -360] * pi / 180, n_steps, 1); repmat([-inf; -inf; 0], n_steps, 1)];
upper_bounds = [repmat([180; 180; 180; 180; 360] * pi / 180, n_steps, 1); repmat([inf; inf; inf], n_steps, 1)];

% --- 求解优化问题 ---
nlp = struct('x', [joint_angles_opt(:); base_pos_opt(:)], 'f', J, 'g', []);
solver = nlpsol('solver', 'ipopt', nlp);
result = solver('x0', initial_guess, 'lbx', lower_bounds, 'ubx', upper_bounds);


% Extract the optimized decision variables
joint_angles_optimized = full(result.x(1:n_steps*n_joints));
joint_angles_optimized = reshape(joint_angles_optimized, n_steps, n_joints);
base_pos_optimized = full(result.x(n_steps*n_joints+1:end));
base_pos_optimized = reshape(base_pos_optimized, n_steps, 3);

keep_base_pos_optimized = base_pos_optimized;
keep_joint_angles_optimized =joint_angles_optimized;
% Number of new time steps to insert between each original time step
m = 5;

% Generate new time vector
old_time_vector = linspace(1, n_steps, n_steps);
new_time_vector = linspace(1, n_steps, m * (n_steps - 1) + n_steps);

% Initialize new joint angles matrix
new_joint_angles = zeros(length(new_time_vector), n_joints);

% Interpolate for each joint
for j = 1:n_joints
    new_joint_angles(:, j) = interp1(old_time_vector, joint_angles_optimized(:, j), new_time_vector, 'spline');
end

% Due to interpolation, we also need to interpolate base positions
base_pos_interpolated = zeros(length(new_time_vector), 3);
for j = 1:3
    base_pos_interpolated(:, j) = interp1(old_time_vector, base_pos_optimized(:, j), new_time_vector, 'linear');
end


% --- Model visualization function ---
hFig = figure; hold on; grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-10, 20]); ylim([-10, 20]); zlim([0, 20]);
view(3);

outputVideo = VideoWriter('robotic_arm_simulation.avi', 'Uncompressed AVI');
open(outputVideo);

% Use the new_joint_angles for visualization
for i = 1:length(new_time_vector)
    % Update robot position
    update_robot(base_pos_interpolated(i, :), [0, 0, 0], new_joint_angles(i, :));
    plot3(desired_trajectory(:, 1), desired_trajectory(:, 2), desired_trajectory(:, 3), 'r--');

    currFrame = getframe(gcf);
    writeVideo(outputVideo, currFrame);

    pause(0.0001);
end

% % --- Model visualization function ---
% hFig = figure; hold on; grid on;
% axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% xlim([-10, 20]); ylim([-10, 20]); zlim([0, 20]);
% view(3);

for i = 1:n_steps
    % Update robot position
    update_robot(base_pos_optimized(i, :), [0, 0, 0], joint_angles_optimized(i, :));
    plot3(desired_trajectory(:, 1), desired_trajectory(:, 2), desired_trajectory(:, 3), 'r--');

    currFrame = getframe(gcf);
    writeVideo(outputVideo, currFrame);
    pause(0.01);
end


close(outputVideo);
function update_robot(base_pos, base_angles, arm_angles)
    cla;  % Clear the current plot but keep the desired trajectory
    
    draw_box(base_pos, [6, 4, 1], base_angles);
    shoulder_pos = base_pos + [0, 0, 2/2];
    % Drawing the arm
    shoulder_pos = base_pos + [0, 0, 2/2];
    [elbow_pos, wrist_pos, end_effector_pos] = compute_arm_positions(shoulder_pos, arm_angles, 5, 4, 2);
    plot3([shoulder_pos(1), elbow_pos(1)], [shoulder_pos(2), elbow_pos(2)], [shoulder_pos(3), elbow_pos(3)], 'k-', 'LineWidth', 2);
    plot3([elbow_pos(1), wrist_pos(1)], [elbow_pos(2), wrist_pos(2)], [elbow_pos(3), wrist_pos(3)], 'b-', 'LineWidth', 2);
    plot3([wrist_pos(1), end_effector_pos(1)], [wrist_pos(2), end_effector_pos(2)], [wrist_pos(3), end_effector_pos(3)], 'g-', 'LineWidth', 2);
end
function draw_box(center, dim, angles)
        dx = dim(1)/2;
        dy = dim(2)/2;
        dz = dim(3)/2;

        corners = [
            center + [-dx -dy -dz];
            center + [-dx -dy +dz];
            center + [-dx +dy -dz];
            center + [-dx +dy +dz];
            center + [+dx -dy -dz];
            center + [+dx -dy +dz];
            center + [+dx +dy -dz];
            center + [+dx +dy +dz];
        ];

        % 将角度转换为矩阵
        R = eul2rotm(angles, 'ZYX');
        for i = 1:8
            corners(i, :) = center + (R * (corners(i, :) - center)')';
        end

        % 定义长方体的6个面
        faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];

        % 绘制长方体
        patch('Vertices', corners, 'Faces', faces, 'FaceColor', 'r');
end

function [elbow_pos, wrist_pos, end_effector_pos] = compute_arm_positions(shoulder_pos, angles, upper_arm_length, forearm_length, wrist_length)
    elbow_pos = shoulder_pos + upper_arm_length * [cos(angles(1)) * cos(angles(2)), sin(angles(1)) * cos(angles(2)), sin(angles(2))];
    wrist_pos = elbow_pos + forearm_length * [cos(angles(1)) * cos(angles(2) + angles(3)), sin(angles(1)) * cos(angles(2) + angles(3)), sin(angles(2) + angles(3))];
    end_effector_pos = wrist_pos + wrist_length * [cos(angles(1)) * cos(angles(2) + angles(3) + angles(4)), sin(angles(1)) * cos(angles(2) + angles(3) + angles(4)), sin(angles(2) + angles(3) + angles(4))];
end

