clear; clc; close all;

% mass
m = [0.2645,0.17,0.1705,0,0,0];
% inertia 
I = cat(3,diag([1.542,0,1.542]*1e-3),diag([0,0.409,0.409]*1e-3),diag([0.413,0.413,0]*1e-3),...
    3*eye(3),2*eye(3),1*eye(3));
% 连杆质心位置(i坐标系下--i坐标系原点->i质心)
r_i_ci = [0,-8.5,0,0,0,0;13.225,0,0,0,0,0;0,3.7,8.525,0,0,0]*1e-2;

%定义几个旋转关节
p560_L(1) = Revolute('d',60.45e-2,	'a',0,      'alpha',-pi/2,  'm',m(1),'r',r_i_ci(:,1),'I',I(:,:,1));
p560_L(2) = Revolute('d',0,	'a',30e-2,	'alpha',0,      'm',m(2),'r',r_i_ci(:,2),'I',I(:,:,2));
p560_L(3) = Revolute('d',0,         'a',0,      'alpha',-pi/2,  'm',m(3),'r',r_i_ci(:,3),'I',I(:,:,3));
p560_L(4) = Revolute('d',34.05e-2,  'a',0,      'alpha',pi/2,   'm',m(4),'r',r_i_ci(:,4),'I',I(:,:,4));
p560_L(5) = Revolute('d',0,         'a',17.05e-2,      'alpha',-pi/2,  'm',m(5),'r',r_i_ci(:,5),'I',I(:,:,5));
p560_L(6) = Revolute('d',40.05e-2,         'a',0,      'alpha',0,      'm',m(6),'r',r_i_ci(:,6),'I',I(:,:,6));
p560 = SerialLink(p560_L, 'name', 'puma560');

% offset
p560.offset = [0,0,0,0,0,0];

% joint variable
% q = [0,0,0,0,0,0];
% qv = [0,0,0,0,0,0];
% qa = [0,0,0,0,0,0];
% he = [0,0,0,0,0,0]';
% 关节角度（单位：弧度）、速度（rad/s）、加速度（rad/s²）、末端外力/力矩
q = [0.2, -0.5, -0.3, -0.6, 0.5, 0.2];
qv = [1, 0.5, -1, 0.3, 0, -1];
qa = [0.2, -0.3, 0.1, 0, -1, 0];
he = [1, 2, -3, -0.5, -2, 1]';

% forward kinematic
T = p560.fkine(q);

% jacobian matrix
J0 = p560.jacob0(zeros(1,6));
J = p560.jacob0(q);

% innverse kinematic
% iter step J\Twist(T).S
% 计算末端 Twist 误差（理想 T 与 q_ikine 正向计算结果的误差）
q_ikine = p560.ikine(T);
err = Twist(T).S;

% 
% T_actual = p560.fkine(q_ikine); % 计算逆解对应的实际位姿
% T_error = T * inv(T_actual);    % 目标T相对于实际T_actual的变换
% err = Twist(T_error).S;         % 这才是严格的位姿误差（旋量形式）
%  代码的简化表达：默认复用 “目标位姿T” 作为误差计算的参考
% 你的代码中err = Twist(T).S是一种 “简写”，其完整逻辑本应是 “目标T与q_ikine对应的实际位姿的误差”。但因为：

% inverse dynamic
torq = p560.rne(q,qv,qa,"fext",[T.R',zeros(3,3);zeros(3,3),T.R']*he);
% 使用逆动力学（Recursive Newton-Euler）计算关节力矩
display(torq);

% plot
figure(1); view(3);
p560.plot(q,"scale",0.5);





hold on;
for i = 1:6
    T_i = p560.A(1:i, q);  % 第 i 个关节的变换矩阵
    Ti = T_i.T;
    origin = Ti(1:3,4);
    x_axis = Ti(1:3,1);
    y_axis = Ti(1:3,2);
    z_axis = Ti(1:3,3);
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 0.05, 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 0.05, 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 0.05, 'b', 'LineWidth', 1.5);
end
legend('X轴','Y轴','Z轴');
