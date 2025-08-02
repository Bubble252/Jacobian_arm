




robot = importrobot('C:\Users\bubble\Desktop\IK_test_recover\matlab\BRX042501\urdf\BRX042501_fixed_wheel_change_axis.urdf');


q   = [0  0   0   0   0    0];%某一瞬时的关节的角度
s=randomConfiguration(robot);
for k = 1:6
    s(k).JointPosition = q(k);
end

robot.show(s)
