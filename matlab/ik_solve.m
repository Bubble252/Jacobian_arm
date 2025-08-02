function [converged, steps, pos_err, att_err, result] = ik_solve(srs, Td, q0)
    % 初始设置
    q = q0;
    q_prev = q;
    dq_prev = zeros(1, 7);

    tol = 1e-4;
    max_iter = 250;
    converged = false;

    pos_errors = zeros(max_iter, 1);
    att_errors = zeros(max_iter, 1);
    dq_norms = zeros(max_iter, 1);
    q_center_dist = zeros(max_iter, 1);
    dq_primary_norm = zeros(max_iter, 1);
    dq_null_norm = zeros(max_iter, 1);

    q_center = (srs.qlim(:,1) + srs.qlim(:,2))' / 2;

    for iter = 1:max_iter
        T = srs.fkine(q);
        T_err = Td / T;
        twist = Twist(T_err).S;
        p_error = twist(1:3);
        w_error = twist(4:6);

        if norm(p_error) < tol && norm(w_error) < tol
            converged = true;
            break;
        end

        error_norm = sqrt(norm(p_error)^2 + norm(w_error)^2);
        % lambda = error_norm < 0.0002 ? 0.003 : 0.005 + 0.05 * (1 - exp(-10 * error_norm));
        lambda = (error_norm < 0.0002) * 0.003 + (error_norm >= 0.0002) * (0.005 + 0.05 * (1 - exp(-10 * error_norm)));

        J = srs.jacob0(q);
        JJT = J * J';
        J_damped = J' / (JJT + lambda^2 * eye(6));

        pos_weight = max(1.0, 8.0 * exp(-8.0 * norm(p_error)));
        att_weight = max(1.0, 5.0 * exp(-8.0 * norm(w_error)));
        weighted_twist = [p_error * pos_weight; w_error * att_weight];
        dq_primary = (J_damped * weighted_twist)';

        task_prev = -(q - q_prev);
        task_center = q_center - q;
        scale = min(1.0, error_norm * 100.0);
        weight_prev = 1.5;
        weight_center = 0.8;
        task_secondary = scale * (weight_prev * task_prev + weight_center * task_center);

        null_proj = eye(7) - J_damped * J;
        dq_null = (null_proj * task_secondary')';
        dq = dq_primary + dq_null;

        max_error = max(norm(p_error), norm(w_error));
        max_step = max(0.0005, 0.08 * min(1.0, max_error * 8.0));
        if norm(dq) > max_step
            dq = dq * (max_step / norm(dq));
        end

        smooth_factor = 0.1;
        dq = (1 - smooth_factor) * dq + smooth_factor * dq_prev;
        dq_prev = dq;

        q_prev = q;
        q = q + dq;

        % 记录数据
        pos_errors(iter) = norm(p_error);
        att_errors(iter) = norm(w_error);
        dq_norms(iter) = norm(dq);
        q_center_dist(iter) = norm(q - q_center);
        dq_primary_norm(iter) = norm(dq_primary);
        dq_null_norm(iter) = norm(dq_null);
        q_history(iter, :) = q;
    end

    steps = iter;

    % 最终误差
    T_final = srs.fkine(q);
    final_twist = Twist(Td / T_final).S;
    pos_err = norm(final_twist(1:3));
    att_err = norm(final_twist(4:6));

    % 输出详细结构（可选）
    if nargout == 5
        result.q_solution = q;
        result.final_pose = T_final;
        result.error_pos = pos_err;
        result.error_att = att_err;
        result.converged = converged;
        result.q_history = q_history(1:iter, :);
        result.pos_errors = pos_errors(1:iter);
        result.att_errors = att_errors(1:iter);
        result.dq_norms = dq_norms(1:iter);
        result.dq_primary_norm = dq_primary_norm(1:iter);
        result.dq_null_norm = dq_null_norm(1:iter);
    end
end
