function srs = create_srs()
    % 创建一个典型的7自由度机械臂模型（可按实际修改 D-H 参数）
    L(1) = Revolute('d', 0.34,   'a', 0,     'alpha', -pi/2);
    L(2) = Revolute('d', 0,      'a', 0,     'alpha',  pi/2);
    L(3) = Revolute('d', 0.4,    'a', 0,     'alpha',  pi/2);
    L(4) = Revolute('d', 0,      'a', 0,     'alpha', -pi/2);
    L(5) = Revolute('d', 0.4,    'a', 0,     'alpha', -pi/2);
    L(6) = Revolute('d', 0,      'a', 0,     'alpha',  pi/2);
    L(7) = Revolute('d', 0.126,  'a', 0,     'alpha', 0);

    srs = SerialLink(L, 'name', 'SRS_7DOF');
end
