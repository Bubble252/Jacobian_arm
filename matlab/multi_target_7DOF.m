num_tests = 20;
success_count = 0;
converge_steps = zeros(num_tests, 1);
final_pos_errors = zeros(num_tests, 1);
final_att_errors = zeros(num_tests, 1);

srs = create_srs();


for i = 1:num_tests
    rpy = rand(1,3) * 2*pi - pi;
    p = [0.2 0.2 0.7] + 0.2 * (rand(1,3)-0.5);
    Td = SE3(rpy2r(rpy), p');

    q0 = (srs.qlim(:,2) - srs.qlim(:,1))' .* rand(1,7) + srs.qlim(:,1)';

    [converged, step, pos_err, att_err] = ik_solve(srs, Td, q0);

    converge_steps(i) = step;
    final_pos_errors(i) = pos_err;
    final_att_errors(i) = att_err;
    success_count = success_count + converged;
end

fprintf("总测试次数: %d，成功次数: %d，成功率: %.2f%%\n", ...
    num_tests, success_count, success_count/num_tests*100);

figure;
histogram(converge_steps);
title('收敛所需步数分布');

figure;
plot(final_pos_errors, 'b.-'); hold on;
plot(final_att_errors, 'r.-');
legend('位置误差', '姿态误差');
title('最终误差分布');
