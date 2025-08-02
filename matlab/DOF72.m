% robot = importrobot('C:\Users\bubble\Desktop\IK_test_recover\matlab\BRX042501\urdf\BRX042501_fixed_wheel_change_axis.urdf');  % 替换成你的URDF文件路径
% robot.DataFormat = 'row';          % 使用行向量表示关节值
% robot.Gravity = [0 0 -9.81];       % 设置重力方向
% figure;
% show(robot);
% title('Imported Robot from URDF');
% config = homeConfiguration(robot);  % 默认位置
% % % 或设置自定义角度，例如：
% % config = [0.2 -0.5 -0.3 0.1 0.3 -0.2];
% show(robot, config);







% robot = importrobot('C:\Users\bubble\Desktop\IK_test_recover\matlab\BRX042501\urdf\BRX042501_fixed_wheel_change_axis.urdf');
% robot.DataFormat = 'row';
% robot.Gravity = [0 0 -9.81];
% 
% baseName = 'ArmR01_Link';     % 替换为你实际的 link 名
% endEffector = 'ArmR08_Link'; % 替换为你实际的末端 link
% 
% % 获取 home 配置
% config = homeConfiguration(robot);
% 
% % 获取从 base → end 的 body/joint 名称链
% bodyList = findBodyPath(robot, baseName, endEffector);
% fprintf('Base to EE path:\n');
% disp(bodyList');
% 
% % 提取中间关节列表
% dh_table = [];
% 
% for i = 2:length(bodyList)
%     body = getBody(robot, bodyList{i});
%     joint = body.Joint;
% 
%     % 获取该关节的静态变换
%     T = joint.JointToParentTransform;
%     [theta, d, a, alpha] = transformToDH(T);
%     dh_table = [dh_table; theta, d, a, alpha];
% 
%     fprintf('[%d] %s: θ=%.3f, d=%.3f, a=%.3f, α=%.3f\n', ...
%         i-1, joint.Name, theta, d, a, alpha);
% end
% 
% disp('--- DH 参数表 ---');
% disp(array2table(dh_table, 'VariableNames', {'theta','d','a','alpha'}));





% 读取原机器人模型
robot = importrobot('C:\Users\bubble\Desktop\IK_test_recover\matlab\BRX042501\urdf\BRX042501_fixed_wheel_change_axis.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

% 设置基座和末端
baseName = 'ArmR01_Link';
endEffector = 'ArmR08_Link';

% 获取从 base → end 的 body 名称链
bodyList = findBodyPath(robot, baseName, endEffector);
fprintf('Base to EE path:\n');
disp(bodyList');

% 构建子树
subRobot = rigidBodyTree('DataFormat','row','Gravity',[0 0 -9.81]);

% 添加 base body（没有 joint）
baseBody = getBody(robot, baseName);
baseBodyCopy = rigidBody(baseBody.Name);  % 不包含 joint
addBody(subRobot, baseBodyCopy, subRobot.BaseName);  % 添加到子树的 base

% 逐个添加中间的 body 和 joint
for i = 2:length(bodyList)
    currentBody = getBody(robot, bodyList{i});
    currentBodyCopy = rigidBody(currentBody.Name);
    currentBodyCopy.Joint = copy(currentBody.Joint);
    parentName = currentBody.Parent.Name;
    addBody(subRobot, currentBodyCopy, parentName);
end

% 显示子链机器人
figure;
show(subRobot);
title(['Subchain: ' baseName ' to ' endEffector]);







function bodyList = findBodyPath(robot, baseName, endEffector)
    % 输出 base 到末端的所有 link 名称
    bodyList = {};
    current = getBody(robot, endEffector);
    while ~strcmp(current.Name, baseName)
        bodyList = [{current.Name}, bodyList];
        parent = current.Parent.Name;  % ✅ 关键改动在这里
        if isempty(parent)
            error('Base link not found in the path to end-effector.');
        end
        current = getBody(robot, parent);
    end
    bodyList = [{baseName}, bodyList];
end


function [theta, d, a, alpha] = transformToDH(T)
    % 假设 T 是 4x4 位姿变换矩阵
    % 返回标准 DH 参数

    % 提取旋转和平移部分
    R = T(1:3,1:3);
    p = T(1:3,4);

    % DH 参数推导（假设 T 是标准形式）
    a = norm(p(1:2));          % 近似 x-y 平面距离作为 a
    d = p(3);                  % z 位移
    alpha = atan2(R(2,3), R(3,3));  % Rz(alpha)
    theta = atan2(R(2,1), R(1,1));  % Rx(theta)
end
