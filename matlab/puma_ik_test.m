clear; clc;

% 构建模型
p560 = create_p560();  % 用你的定义构造函数或者直接复制那一段定义
q = zeros(1, 6);        % 初始关节角度
q_prev = q;
dq_prev = zeros(1, 6);

% % 目标位姿 Td
% rpy = [0.2 0 0.3];              % RPY姿态
% p = [0.2 0.2 0.7];              % 位置
% Rd = rpy2r(rpy);                % 旋转矩阵
% Td = rt2tr(Rd, p');             % 目标变换矩阵
% 修改为构造 SE3 类型的 Td
% rpy = [0.2 0 0.3];              % RPY姿态
% p = [0.2 0.2 0.7];              % 位置
% Td = SE3(rpy, 'rpy') * SE3(p);  % 直接用 SE3 构造（先旋转后平移，和你原逻辑一致）
% 目标位姿 Td（修改后）
rpy = [0.2 0 0.3];              % RPY姿态
p = [0.2 0.2 0.7];              % 位置
Rd = rpy2r(rpy);                % 旋转矩阵（3x3）
Td = SE3(Rd, p');               % 用旋转矩阵+平移向量构造 SE3（平移向量需为列向量）

% 迭代参数
tol = 1e-4;
max_iter = 200;
lambda_base = 0.01;    % 初始阻尼
converged = false;

fprintf("iter\tpos_err\t\tatt_err\t\tcond\t\tdq_norm\n");




figure(1);
p560.plot(q, 'workspace', [-1 1 -1 1 0 1.5], 'scale', 0.5);
hold on;
trplot(Td, 'frame', 'T_d', 'color', 'red', 'length', 0.2); % 显示目标

pos_errors = [];
att_errors = [];
dq_norms = [];

actual_traj = [];
target_traj = [];




for iter = 1:max_iter
    % 当前正解
    T = p560.fkine(q);

%     disp('Td 的类型：'); whos Td
% disp('T 的类型：'); whos T
% disp('Td 的内容：'); disp(Td)
% disp('T 的内容：'); disp(T)
    
    % twist误差
    T_err = Td / T;              % 目标变换 * 当前变换的逆
    twist = Twist(T_err).S;      % twist 向量
    p_error = twist(1:3);
    w_error = twist(4:6);

    % 收敛判断
    if norm(p_error) < tol && norm(w_error) < tol
        converged = true;
        break;
    end

    % 阻尼调整
    error_norm = sqrt(norm(p_error)^2 + norm(w_error)^2);
    if error_norm < 0.0002
        lambda = 0.003;
    else
        lambda = 0.005 + 0.05 * (1 - exp(-10 * error_norm));
    end

    % 雅可比与伪逆
    J = p560.jacob0(q);
    cond_J = cond(J);
    JJT = J * J';
    J_damped = J' / (JJT + lambda^2 * eye(6));  % 阻尼伪逆

    % 主任务：末端位姿误差
    pos_weight = max(1.0, 8.0 * exp(-8.0 * norm(p_error)));
    att_weight = max(1.0, 5.0 * exp(-8.0 * norm(w_error)));
    weighted_twist = [p_error * pos_weight; w_error * att_weight];
    dq_primary = (J_damped * weighted_twist)';

    % 次任务：靠近 q_prev + 向中心靠拢
    q_center = (p560.qlim(:,1) + p560.qlim(:,2))' / 2;
    task_prev = -(q - q_prev);
    task_center = q_center - q;

    weight_prev = 1.5;
    weight_center = 0.8;
    scale = min(1.0, error_norm * 100.0);
    task_secondary = scale * (weight_prev * task_prev + weight_center * task_center);

    % 零空间投影
    null_proj = eye(6) - J_damped * J;
    dq_null = (null_proj * task_secondary')';

    % 合并主任务与零空间任务
    dq = dq_primary + dq_null;

    % 步长限制
    max_error = max(norm(p_error), norm(w_error));
    max_step = max(0.0005, 0.08 * min(1.0, max_error * 8.0));
    if norm(dq) > max_step
        dq = dq * (max_step / norm(dq));
    end

    % 平滑（减少抖动）
    smooth_factor = 0.1;
    dq = (1 - smooth_factor) * dq + smooth_factor * dq_prev;
    dq_prev = dq;

    % 更新
    q_prev = q;
    q = q + dq;


    p560.animate(q);


    ee_pos = transl(T);            % 实际末端位置
    target_pos = transl(Td);       % 目标末端位置

    actual_traj(end+1, :) = ee_pos;
    target_traj(end+1, :) = target_pos;
    pause(0.01);  % 控制动画速度

    pos_errors(end+1) = norm(p_error);
    att_errors(end+1) = norm(w_error);
    dq_norms(end+1) = norm(dq);



    fprintf("%3d\t%8.5f\t%8.5f\t%8.2f\t%8.5f\n", iter, norm(p_error), norm(w_error), cond_J, norm(dq));
end

% 最终验证
T_final = p560.fkine(q);
T_err = Td / T_final;
final_twist = Twist(T_err).S;

fprintf("\n最终误差:\n");
fprintf("位置误差范数: %.6f\n", norm(final_twist(1:3)));
fprintf("姿态误差范数: %.6f\n", norm(final_twist(4:6)));
fprintf("是否收敛: %s\n", string(converged));


figure;
subplot(3,1,1);
plot(pos_errors, 'b'); ylabel('位置误差');
subplot(3,1,2);
plot(att_errors, 'r'); ylabel('姿态误差');
subplot(3,1,3);
plot(dq_norms, 'k'); ylabel('dq范数'); xlabel('迭代步');
sgtitle('逆运动学迭代过程');



figure;
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('末端轨迹与目标对比');

% 连线
plot3(actual_traj(:,1), actual_traj(:,2), actual_traj(:,3), 'b-', 'LineWidth', 2);
plot3(target_traj(:,1), target_traj(:,2), target_traj(:,3), 'r--', 'LineWidth', 2);

% 起点与终点
scatter3(actual_traj(1,1), actual_traj(1,2), actual_traj(1,3), 80, 'bo', 'filled');
scatter3(actual_traj(end,1), actual_traj(end,2), actual_traj(end,3), 80, 'b^', 'filled');
scatter3(target_traj(1,1), target_traj(1,2), target_traj(1,3), 80, 'ro', 'filled');
scatter3(target_traj(end,1), target_traj(end,2), target_traj(end,3), 80, 'r^', 'filled');

% 绘制误差箭头（每隔几步画一个）
for i = 1:5:length(actual_traj)
    quiver3( ...
        target_traj(i,1), target_traj(i,2), target_traj(i,3), ...
        actual_traj(i,1) - target_traj(i,1), ...
        actual_traj(i,2) - target_traj(i,2), ...
        actual_traj(i,3) - target_traj(i,3), ...
        0, 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.5 ...
    );
end

legend('实际轨迹', '目标轨迹', 'Location', 'best');




function p560 = create_p560()
    m = [0.2645,0.17,0.1705,0,0,0];
    I = cat(3,diag([1.542,0,1.542]*1e-3),diag([0,0.409,0.409]*1e-3),diag([0.413,0.413,0]*1e-3),...
        3*eye(3),2*eye(3),1*eye(3));
    r_i_ci = [0,-8.5,0,0,0,0;13.225,0,0,0,0,0;0,3.7,8.525,0,0,0]*1e-2;

    p560_L(1) = Revolute('d',60.45e-2,	'a',0,      'alpha',-pi/2,  'm',m(1),'r',r_i_ci(:,1),'I',I(:,:,1));
    p560_L(2) = Revolute('d',0,	'a',30e-2,	'alpha',0,      'm',m(2),'r',r_i_ci(:,2),'I',I(:,:,2));
    p560_L(3) = Revolute('d',0,         'a',0,      'alpha',-pi/2,  'm',m(3),'r',r_i_ci(:,3),'I',I(:,:,3));
    p560_L(4) = Revolute('d',34.05e-2,  'a',0,      'alpha',pi/2,   'm',m(4),'r',r_i_ci(:,4),'I',I(:,:,4));
    p560_L(5) = Revolute('d',0,         'a',17.05e-2,      'alpha',-pi/2,  'm',m(5),'r',r_i_ci(:,5),'I',I(:,:,5));
    p560_L(6) = Revolute('d',40.05e-2,         'a',0,      'alpha',0,      'm',m(6),'r',r_i_ci(:,6),'I',I(:,:,6));
    p560 = SerialLink(p560_L, 'name', 'puma560');
end
