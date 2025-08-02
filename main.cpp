
#include "robotics.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#ifndef PI
#define PI 3.141592653589793f
#endif

// 调试开关
#define DEBUG_MODE 1
#define SAVE_ITERATION_LOG 0
#define CHECK_WORKSPACE 1
#define AVOID_SINGULARITIES 1  // 启用奇异解规避

// 零空间任务权重（可根据需求调整）
#define WEIGHT_PREV_POS 1.6f    // 接近上一位置的权重
#define WEIGHT_CENTER 0.4f      // 向关节范围中心移动的权重

// 计算矩阵条件数（用于监测雅可比矩阵奇异性）
template <int R, int C>
float conditionNumber(const Matrixf<R, C>& mat) {
    float max = 0, min = 1e9;
    for (int i = 0; i < std::min(R, C); i++) {
        float val = std::abs(mat[i][i]);
        if (val > max) max = val;
        if (val < min && val > 1e-9) min = val;
    }
    return max / min;
}

// 验证点是否在工作空间内
bool isPointInWorkspace(const robotics::Serial_Link<7>& robot, const Matrixf<3, 1>& p) {
    float min_z = 0.34f;  // 基座高度
    float max_z = 0.34f + 0.4f + 0.39f + 0.126f;  // 最大可达高度
    float max_r = 0.4f + 0.4f + 0.126f;  // 最大水平半径
    
    float x = p[0][0], y = p[1][0], z = p[2][0];
    float r = std::sqrt(x*x + y*y);
    
    return (z >= min_z && z <= max_z && r <= max_r);
}

// 计算雅可比矩阵的伪逆和零空间投影矩阵
template <int R, int C>
struct JacobianInfo {
    Matrixf<C, R> pinv;      // 伪逆
    Matrixf<C, C> null_proj; // 零空间投影矩阵
    float cond;             // 条件数
    float min_sv;           // 最小奇异值
};

template <int R, int C>
JacobianInfo<R, C> computeJacobianInfo( Matrixf<R, C>& J, float lambda) {
    JacobianInfo<R, C> info;
    
    // 计算雅可比矩阵条件数（简化版）
    info.cond = conditionNumber(J.trans() * J);
    
    // 计算阻尼最小二乘伪逆 J† = (JᵀJ + λ²I)⁻¹Jᵀ
    Matrixf<C, C> JTJ = J.trans() * J;
    Matrixf<C, C> damping  = lambda * lambda * matrixf::eye<C, C>();
    info.pinv = matrixf::inv(JTJ + damping) * J.trans();
    
    // 计算零空间投影矩阵 P = I - J†J
    info.null_proj = matrixf::eye<C, C>() - info.pinv * J;
    
    // 估计最小奇异值（用于奇异检测）
    info.min_sv = 1.0f / std::sqrt(info.cond);
    
    return info;
}

int main() {
    // 定义7DOF机械臂DH参数
    robotics::Link links[7] = {
        robotics::Link(0.0f, 0.34f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
        robotics::Link(0.0f, 0.0f, 0.4f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
        robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
        robotics::Link(0.0f, 0.0f, 0.39f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
        robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
        robotics::Link(0.0f, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
        robotics::Link(0.0f, 0.126f, 0.0f, 0.0f, robotics::R, 0.0f, -PI, PI, 1.0f,
                       matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>())
    };

    robotics::Serial_Link<7> robot(links);
    Matrixf<7, 1> q_prev = matrixf::zeros<7, 1>();  // 上一时刻关节角度

    // 目标位姿（确保在工作空间内）
    float rpy_data[3] = {0.2f, 0.0f, 0.3f};  // RPY姿态
    float p_data[3] = {0.2f, 0.2f, 0.7f};    // 位置（z≥0.34）
    Matrixf<3, 1> rpy(rpy_data);
    Matrixf<3, 1> p(p_data);
    Matrixf<4, 4> Td = robotics::rp2t(robotics::rpy2r(rpy), p);

    // 工作空间验证
    #if CHECK_WORKSPACE
    std::cout << "目标位置: (" << p[0][0] << ", " << p[1][0] << ", " << p[2][0] << ")\n";
    if (!isPointInWorkspace(robot, p)) {
        std::cout << "错误：目标点超出工作空间！\n";
        return -1;
    }
    #endif

    // 逆运动学求解
    float tol = 1e-4f;
    uint16_t max_iter = 200;
    Matrixf<7, 1> q = q_prev;  // 初始关节角度
    bool converged = false;
    Matrixf<7, 1> dq_prev = matrixf::zeros<7, 1>();

    // 零空间任务权重（可根据需求调整）
    const float weight_prev = 1.5f;   // 接近上一位置的权重（更高优先级）
    const float weight_center = 0.8f; // 向关节中心移动的权重（较低优先级）

    std::cout << "\n开始迭代求解...\n";
    std::cout << "迭代次数 | 位置误差范数 | 姿态误差范数 | 雅可比条件数 | 关节角度变化范数\n";
    std::cout << "------------------------------------------------------------------\n";
    
    for (uint16_t iter = 0; iter < max_iter; iter++) {
        // 计算当前位姿和误差
        Matrixf<4, 4> T = robot.fkine(q);
        Matrixf<3, 1> p_error = robotics::t2p(Td) - robotics::t2p(T);
        Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T));
        Matrixf<3, 1> w_error;
        for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];
        Matrixf<6, 7> J = robot.jacob(q);

        // 检查收敛
        if (p_error.norm() < tol && w_error.norm() < tol) {
            converged = true;
            break;
        }

        // 1. 动态阻尼系数（基于误差和奇异值）
        float error_norm = std::sqrt(p_error.norm() + w_error.norm());
        // float base_lambda = 0.005f + 0.05f * (1.0f - std::exp(-10.0f * error_norm));



        //新增？？??????????????????????????????????????????????????
        float base_lambda;
        if (error_norm < 0.0002f) {  // 误差接近阈值时
        base_lambda = 0.003f;  // 减小阻尼，增强主任务调整
        } else {
        base_lambda = 0.005f + 0.05f * (1.0f - std::exp(-10.0f * error_norm));
        }
        
        // 2. 奇异值检测与增强阻尼
        float lambda = base_lambda;
        #if AVOID_SINGULARITIES
        JacobianInfo<6, 7> jac_info = computeJacobianInfo(J, base_lambda);
        float cond = jac_info.cond;
        float min_sv = jac_info.min_sv;
        
        // 接近奇异位形时增大阻尼
        if (min_sv < 0.05f) {
            lambda = base_lambda + 0.5f * (0.05f - min_sv);  // 奇异时阻尼增大
            std::cout << "警告：接近奇异位形！条件数=" << cond << ", 阻尼=" << lambda << "\n";
        } else {
            cond = conditionNumber(J.trans() * J);
        }
        #else
        float cond = conditionNumber(J.trans() * J);
        #endif

        // 3. 动态误差权重
        float pos_weight = std::max(1.0f, 8.0f * std::exp(-8.0f * p_error.norm()));
        float att_weight = std::max(1.0f, 5.0f * std::exp(-8.0f * w_error.norm()));
        Matrixf<6, 1> weighted_err;
        for (int i = 0; i < 3; i++) weighted_err[i][0] = p_error[i][0] * pos_weight;
        for (int i = 3; i < 6; i++) weighted_err[i][0] = w_error[i-3][0] * att_weight;

        // 4. 零空间优化：组合任务（带权重）
        Matrixf<7, 1> primary_task = matrixf::zeros<7, 1>();
        Matrixf<7, 1> secondary_task = matrixf::zeros<7, 1>();
        
        // 主任务：末端执行器位姿跟踪
        primary_task = computeJacobianInfo(J, lambda).pinv * weighted_err;
        
        // 次任务1：接近上一位置（q_prev）- 权重更高
        Matrixf<7, 1> task_prev = -(q - q_prev);  // 偏差越小，任务量越小
        
        // 次任务2：向关节范围中心移动 - 权重较低
        Matrixf<7, 1> task_center = matrixf::zeros<7, 1>();
        for (int i = 0; i < 7; i++) {
            float q_center = (links[i].qmin() + links[i].qmax()) / 2.0f;  // 关节范围中心
            task_center[i][0] = (q_center - q[i][0]);  // 向中心吸引
        }

        // 合并次任务（带权重）
        // secondary_task = weight_prev * task_prev + weight_center * task_center;
        
        //新增====================================================================================
        float weight_scale = std::min(1.0f, error_norm * 100.0f);  // 误差越小，权重缩放系数越小
        float current_weight_prev = weight_prev * weight_scale;
        float current_weight_center = weight_center * weight_scale;
        secondary_task = current_weight_prev * task_prev + current_weight_center * task_center;



        // 奇异规避增强（接近奇异时补充梯度项）
        #if AVOID_SINGULARITIES
        if (min_sv < 0.05f) {
            // 计算雅可比行列式梯度（规避奇异）
            Matrixf<7, 1> task_singular = matrixf::zeros<7, 1>();
            for (int i = 0; i < 7; i++) {
                Matrixf<7, 1> q_perturbed = q;
                q_perturbed[i][0] += 0.001f;  // 微小扰动
                Matrixf<6, 7> J_perturbed = robot.jacob(q_perturbed);
                float cond_perturbed = conditionNumber(J_perturbed.trans() * J_perturbed);
                
                // 沿减小条件数的方向调整（增强奇异规避）
                task_singular[i][0] = 0.1f * (cond - cond_perturbed);
            }
            // 奇异规避任务权重（可单独调整）
            secondary_task += 0.3f * task_singular;
        }
        #endif

        // 5. 零空间投影：次任务不影响主任务
        Matrixf<7, 1> null_proj = matrixf::zeros<7, 1>();
        #if AVOID_SINGULARITIES
        null_proj = jac_info.null_proj * secondary_task;
        #else
        Matrixf<7, 7> null_space = matrixf::eye<7, 7>() - computeJacobianInfo(J, lambda).pinv * J;
        null_proj = null_space * secondary_task;
        #endif

        // 6. 合并主任务和零空间优化
        Matrixf<7, 1> dq = primary_task + null_proj;

        // 7. 动态步长限制
        
        // float max_step = 0.08f * std::min(1.0f, std::max(p_error.norm(), w_error.norm()) * 15.0f);

        // 7. 优化动态步长限制
        float max_error = std::max(p_error.norm(), w_error.norm());
        float base_step = 0.08f;
        float error_scale = std::min(1.0f, max_error * 8.0f);
        float max_step = base_step * error_scale;
        max_step = std::max(max_step, 0.0005f);  // 确保最小步长

        float dq_norm = dq.norm();
        if (dq_norm > max_step) dq = dq * (max_step / dq_norm);

        // 8. 平滑项（减少抖动）
        float smooth_factor = 0.2f;
        dq = (1 - smooth_factor) * dq + smooth_factor * dq_prev;
        dq_prev = dq;

        // 更新关节角度（保存当前q为下一迭代的q_prev）
        q_prev = q;  // 先保存当前位置作为下一迭代的"上一位置"
        q = q + dq;

        // 输出迭代信息
        #if DEBUG_MODE
        std::cout << std::setw(4) << iter << "    | " 
                  << std::setw(12) << p_error.norm() << " | " 
                  << std::setw(12) << w_error.norm() << " | " 
                  << std::setw(12) << cond << " | "
                  << std::setw(14) << dq.norm() << "\n";
        #endif
    }

    // 结果验证与输出
    Matrixf<4, 4> T_result = robot.fkine(q);
    Matrixf<3, 1> p_result = robotics::t2p(T_result);
    Matrixf<3, 1> p_error = robotics::t2p(Td) - p_result;
    Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T_result));
    Matrixf<3, 1> w_error;
    for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];

    std::cout << "\n最终结果:\n";
    std::cout << "位置误差范数: " << p_error.norm() << " (目标<" << tol << ")\n";
    std::cout << "姿态误差范数: " << w_error.norm() << " (目标<" << tol << ")\n";
    std::cout << "是否收敛: " << (converged ? "是" : "否") << "\n";

    return 0;
}




























// /**
//  ******************************************************************************
//  * @file    main_enhanced.cpp
//  * @brief   增强版 7DOF 机械臂逆运动学 (结合原始代码优点)
//  * @author  Spoon Guan
//  * @date    2025-07-26
//  ******************************************************************************
//  */

// #include "robotics.h"
// #include <iostream>
// #include <iomanip>
// #include <fstream>
// #include <cmath>

// #ifndef PI
// #define PI 3.141592653589793f
// #endif

// // 调试开关
// #define DEBUG_MODE 1
// #define CHECK_WORKSPACE 1
// #define AVOID_SINGULARITIES 1  // 启用奇异解规避

// // 计算矩阵条件数（用于监测雅可比矩阵奇异性）
// template <int R, int C>
// float conditionNumber(const Matrixf<R, C>& mat) {
//     float max_val = 0, min_val = 1e9;
//     for (int i = 0; i < std::min(R, C); i++) {
//         float val = std::abs(mat[i][i]);
//         if (val > max_val) max_val = val;
//         if (val < min_val && val > 1e-9) min_val = val;
//     }
//     return (min_val > 1e-9) ? (max_val / min_val) : 1e9;
// }

// // 验证点是否在工作空间内
// bool isPointInWorkspace(robotics::Serial_Link<7>& robot, const Matrixf<3, 1>& p) {
//     float min_z = 0.34f;  // 基座高度
//     float max_z = 0.34f + 0.4f + 0.39f + 0.126f;  // 最大可达高度
//     float max_r = 0.4f + 0.4f + 0.126f;  // 最大水平半径
    
//     float x = p[0][0], y = p[1][0], z = p[2][0];
//     float r = std::sqrt(x*x + y*y);
    
//     return (z >= min_z && z <= max_z && r <= max_r);
// }

// // 增强版逆运动学求解器（结合原始代码优点）
// Matrixf<7, 1> ikine_enhanced(
//     robotics::Serial_Link<7>& robot,
//     robotics::Link links[7],
//     const Matrixf<4, 4>& Td,
//     const Matrixf<7, 1>& q_prev,
//     float tol = 1e-4f,
//     int max_iter = 150
// ) {
//     Matrixf<7, 1> q = q_prev;
//     Matrixf<7, 7> I = matrixf::eye<7, 7>();
    
//     // 关键参数
//     float alpha = 0.03f;        // 零空间项权重（适度减小以减少振荡）
//     float lambda_base = 0.008f; // 基础阻尼系数
//     float dq_max = 0.05f;       // 最大步长限制（减小以提高稳定性）
    
//     // 平滑和振荡抑制参数
//     Matrixf<7, 1> dq_prev = matrixf::zeros<7, 1>();
//     float smooth_factor = 0.3f;  // 平滑系数
    
//     // 振荡检测
//     Matrixf<3, 1> pe_history[3];
//     Matrixf<3, 1> we_history[3];
//     int history_idx = 0;
//     bool oscillation_detected = false;
    
//     std::cout << "\n开始增强版迭代求解...\n";
//     std::cout << "迭代次数 | 位置误差范数 | 姿态误差范数 | 阻尼系数   | 条件数     | 状态\n";
//     std::cout << "-------------------------------------------------------------------------\n";
    
//     for (int iter = 0; iter < max_iter; ++iter) {
//         // 1. 计算当前位姿和误差
//         Matrixf<4, 4> T = robot.fkine(q);
//         Matrixf<3, 1> pe = robotics::t2p(Td) - robotics::t2p(T);
//         Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T));
//         Matrixf<3, 1> we;
//         for (int j = 0; j < 3; ++j) we[j][0] = twist[j + 3][0];
        
//         // 2. 振荡检测（每3次迭代检查一次）
//         if (iter >= 6) {
//             float pe_var = 0, we_var = 0;
//             for (int i = 0; i < 3; i++) {
//                 pe_var += std::abs(pe_history[i].norm() - pe.norm());
//                 we_var += std::abs(we_history[i].norm() - we.norm());
//             }
            
//             // 如果误差在小范围内振荡，启用强阻尼模式
//             if (pe_var < tol * 5 && we_var < tol * 5 && (pe.norm() + we.norm()) > tol * 2) {
//                 oscillation_detected = true;
//             }
//         }
        
//         // 更新历史记录
//         pe_history[history_idx % 3] = pe;
//         we_history[history_idx % 3] = we;
//         history_idx++;
        
//         // 3. 检查收敛条件
//         if (pe.norm() < tol && we.norm() < tol) {
//             std::cout << "收敛成功！迭代次数: " << iter << "\n";
//             return q;
//         }
        
//         // 4. 计算雅可比矩阵和条件数
//         Matrixf<6, 7> J = robot.jacob(q);
//         Matrixf<7, 7> JTJ = J.trans() * J;
//         float cond = conditionNumber(JTJ);
        
//         // 5. 动态阻尼系数（结合原始代码优点）
//         float pe_norm = pe.norm();
//         float we_norm = we.norm();
//         float total_error = std::sqrt(pe_norm * pe_norm + we_norm * we_norm);
        
//         // 基础动态阻尼
//         float lambda = lambda_base + 0.015f * (1.0f - std::exp(-10.0f * total_error));
        
//         // 奇异性检测与增强阻尼
//         #if AVOID_SINGULARITIES
//         float min_sv_estimate = 1.0f / std::sqrt(std::max(cond, 1.0f));
//         if (min_sv_estimate < 0.08f || cond > 500.0f) {
//             lambda += 0.3f * (0.08f - std::min(min_sv_estimate, 0.08f));
//             std::cout << " [奇异规避]";
//         }
//         #endif
        
//         // 振荡抑制：如果检测到振荡，大幅增加阻尼
//         if (oscillation_detected) {
//             lambda += 0.5f;
//             alpha *= 0.5f;  // 同时减小零空间项权重
//             std::cout << " [振荡抑制]";
//         }
        
//         // 6. 动态误差权重（结合原始代码优点）
//         float pos_weight = std::max(1.0f, 8.0f * std::exp(-8.0f * pe_norm));
//         float att_weight = std::max(1.0f, 5.0f * std::exp(-8.0f * we_norm));
        
//         Matrixf<6, 1> err;
//         for (int j = 0; j < 3; ++j) {
//             err[j][0] = pe[j][0] * pos_weight;
//             err[j + 3][0] = we[j][0] * att_weight;
//         }
        
//         // 7. 零空间优化项：k = -alpha * (q - q_prev)
//         Matrixf<7, 1> k = -alpha * (q - q_prev);
        
//         // 8. 统一的阻尼最小二乘求解
//         Matrixf<7, 1> rhs = J.trans() * err + k;
//         Matrixf<7, 1> dq = matrixf::inv(JTJ + lambda * I) * rhs;
        
//         // 9. 自适应步长控制（结合原始代码优点）
//         float dq_norm = dq.norm();
//         float adaptive_max_step = dq_max * std::min(1.0f, std::max(pe_norm, we_norm) * 20.0f);
        
//         // 振荡时进一步减小步长
//         if (oscillation_detected) {
//             adaptive_max_step *= 0.3f;
//         }
        
//         if (dq_norm > adaptive_max_step) {
//             dq = dq * (adaptive_max_step / dq_norm);
//         }
        
//         // 10. 平滑项（结合原始代码优点）
//         dq = (1 - smooth_factor) * dq + smooth_factor * dq_prev;
//         dq_prev = dq;
        
//         // 11. 更新关节角度
//         q = q + dq;
        
//         // 12. 关节限位约束
//         for (int j = 0; j < 7; ++j) {
//             if (links[j].type() == robotics::R) {
//                 // 角度标准化到[-π, π]
//                 while (q[j][0] > PI) q[j][0] -= 2 * PI;
//                 while (q[j][0] < -PI) q[j][0] += 2 * PI;
                
//                 // 关节限位约束
//                 if (q[j][0] < links[j].qmin()) q[j][0] = links[j].qmin();
//                 if (q[j][0] > links[j].qmax()) q[j][0] = links[j].qmax();
//             }
//         }
        
//         // 13. 输出迭代信息
//         #if DEBUG_MODE
//         std::cout << std::setw(4) << iter << "    | " 
//                   << std::setw(12) << std::setprecision(6) << pe.norm() << " | " 
//                   << std::setw(12) << std::setprecision(6) << we.norm() << " | " 
//                   << std::setw(8) << std::setprecision(4) << lambda << " | "
//                   << std::setw(8) << std::setprecision(1) << cond << " | ";
        
//         if (oscillation_detected) std::cout << "振荡抑制";
//         else if (min_sv_estimate < 0.08f) std::cout << "奇异规避";
//         else std::cout << "正常";
        
//         std::cout << "\n";
//         #endif
        
//         // 重置振荡检测标志（避免永久激活）
//         if (iter % 10 == 0) {
//             oscillation_detected = false;
//             alpha = 0.03f;  // 恢复原始零空间权重
//         }
//     }
    
//     std::cout << "警告：未在最大迭代次数内收敛！\n";
//     return q;
// }

// int main() {
//     // 定义7DOF机械臂DH参数
//     robotics::Link links[7] = {
//         robotics::Link(0.0f, 0.34f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.4f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.39f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.126f, 0.0f, 0.0f, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>())
//     };

//     robotics::Serial_Link<7> robot(links);
//     Matrixf<7, 1> q_prev = matrixf::zeros<7, 1>();

//     // 目标位姿（确保在工作空间内）
//     float rpy_data[3] = {0.1f, 0.2f, 0.3f};  // RPY姿态
//     float p_data[3] = {0.2f, 0.2f, 0.5f};    // 位置
//     Matrixf<3, 1> rpy(rpy_data);
//     Matrixf<3, 1> p(p_data);
//     Matrixf<4, 4> Td = robotics::rp2t(robotics::rpy2r(rpy), p);

//     // 工作空间验证
//     #if CHECK_WORKSPACE
//     std::cout << "目标位置: (" << p[0][0] << ", " << p[1][0] << ", " << p[2][0] << ")\n";
//     if (!isPointInWorkspace(robot, p)) {
//         std::cout << "错误：目标点超出工作空间！\n";
//         return -1;
//     }
//     #endif

//     // 调用增强版求解器
//     float tol = 1e-4f;
//     int max_iter = 150;
//     Matrixf<7, 1> q_result = ikine_enhanced(robot, links, Td, q_prev, tol, max_iter);

//     // 结果验证
//     Matrixf<4, 4> T_result = robot.fkine(q_result);
//     Matrixf<3, 1> p_result = robotics::t2p(T_result);
//     Matrixf<3, 1> p_error = robotics::t2p(Td) - p_result;
//     Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T_result));
//     Matrixf<3, 1> w_error;
//     for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];

//     std::cout << "\n=== 最终结果 ===\n";
//     std::cout << "位置误差范数: " << p_error.norm() << "\n";
//     std::cout << "姿态误差范数: " << w_error.norm() << "\n";
//     std::cout << "是否收敛: " << ((p_error.norm() < tol && w_error.norm() < tol) ? "是" : "否") << "\n";
    
//     std::cout << "\n关节角度 (度):\n";
//     for (int i = 0; i < 7; i++) {
//         std::cout << "q" << i+1 << ": " << std::setprecision(3) << q_result[i][0] * 180.0f / PI << "°\n";
//     }

//     return 0;
// }













// /**
//  ******************************************************************************
//  * @file    main.cpp
//  * @brief   测试 7DOF 机械臂逆运动学
//  * @author  Spoon Guan
//  * @date    2025-07-25
//  ******************************************************************************
//  * Copyright (c) 2023 Team JiaoLong-SJTU
//  * All rights reserved.
//  ******************************************************************************
//  */

// #include "robotics.h"
// #include <iostream>
// #include <iomanip>

// // 补充 PI 定义（若 robotics.hpp 中未声明）
// #ifndef PI
// #define PI 3.141592653589793f
// #endif

// int main() {
//     // 定义 7DOF 机械臂的 DH 参数
//     robotics::Link links[7] = {
//         robotics::Link(0.0f, 0.34f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.4f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.39f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.126f, 0.0f, 0.0f, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>())
//     };

//     // 初始化机械臂
//     robotics::Serial_Link<7> robot(links);

//     // 初始关节角度（全零）
//     Matrixf<7, 1> q_prev = matrixf::zeros<7, 1>();

//     // 目标位姿
//     float rpy_data[3] = {0.1f, 0.2f, 0.3f}; // [yaw, pitch, roll]
//     float p_data[3] = {0.2f, 0.2f, 0.2f};   // [px, py, pz]
//     Matrixf<3, 1> rpy(rpy_data);
//     Matrixf<3, 1> p(p_data);
//     Matrixf<4, 4> Td = robotics::rp2t(robotics::rpy2r(rpy), p);

//     // 逆运动学求解（调用修正后的 ikine_smooth）
//     float tol = 1e-4f; // 容差
//     uint16_t max_iter = 100; // 最大迭代次数
//     Matrixf<7, 1> q = robot.ikine_smooth(Td, q_prev, tol, max_iter);

//     // 验证结果
//     Matrixf<4, 4> T_result = robot.fkine(q);
//     Matrixf<3, 1> p_error = robotics::t2p(Td) - robotics::t2p(T_result);
//     Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T_result));
//     Matrixf<3, 1> w_error;
//     for (int i = 0; i < 3; ++i) {
//         w_error[i][0] = twist[i + 3][0]; // 提取姿态误差
//     }

//     // 输出结果
//     std::cout << std::fixed << std::setprecision(6);
//     std::cout << "逆运动学结果 (关节角度, 单位: rad):\n";
//     for (int i = 0; i < 7; ++i) {
//         std::cout << "q[" << i << "] = " << q[i][0];
//         // 检查关节角度是否在范围内（调用 const 版本的 qmin/qmax）
//         if (q[i][0] < links[i].qmin() || q[i][0] > links[i].qmax()) {
//             std::cout << " (超出范围: [" << links[i].qmin() << ", " << links[i].qmax() << "])";
//         }
//         std::cout << "\n";
//     }
//     std::cout << "位置误差范数: " << p_error.norm() << "\n";
//     std::cout << "姿态误差范数: " << w_error.norm() << "\n";
//     std::cout << "逆运动学是否收敛: " << (p_error.norm() < tol && w_error.norm() < tol ? "是" : "否") << "\n";

//     return 0;
// }



// /**
//  ******************************************************************************
//  * @file    main.cpp
//  * @brief   测试 7DOF 机械臂逆运动学 (增强调试版本)
//  * @author  Spoon Guan
//  * @date    2025-07-25
//  ******************************************************************************
//  * Copyright (c) 2023 Team JiaoLong-SJTU
//  * All rights reserved.
//  ******************************************************************************
//  */

// #include "robotics.h"
// #include <iostream>
// #include <iomanip>
// #include <fstream>
// #include <cmath>

// // 补充 PI 定义（若 robotics.hpp 中未声明）
// #ifndef PI
// #define PI 3.141592653589793f
// #endif

// // 调试开关
// #define DEBUG_MODE 1          // 启用详细调试输出
// #define SAVE_ITERATION_LOG 0  // 保存迭代过程到文件
// #define CHECK_WORKSPACE 1     // 验证目标点是否在工作空间内

// // 计算矩阵条件数（用于监测雅可比矩阵奇异性）
// template <int R, int C>
// float conditionNumber(const Matrixf<R, C>& mat) {
//     // 简化版：计算对角元素比值作为条件数估计
//     float max = 0, min = 1e9;
//     for (int i = 0; i < std::min(R, C); i++) {
//         float val = std::abs(mat[i][i]);
//         if (val > max) max = val;
//         if (val < min && val > 1e-9) min = val; // 避免除以零
//     }
//     return max / min;
// }

// // 验证点是否在工作空间内（简化版）
// bool isPointInWorkspace(const robotics::Serial_Link<7>& robot, const Matrixf<3, 1>& p) {
//     // 计算机械臂可达范围边界
//     float min_z = 0.34f;  // 基于DH参数的最小z（基座高度）
//     float max_z = 0.34f + 0.4f + 0.39f + 0.126f;  // 各连杆z方向叠加
//     float max_r = 0.4f + 0.4f + 0.126f;  // 水平方向最大伸展
    
//     float x = p[0][0], y = p[1][0], z = p[2][0];
//     float r = std::sqrt(x*x + y*y);
    
//     return (z >= min_z && z <= max_z && r <= max_r);
// }

// int main() {
//     // 定义 7DOF 机械臂的 DH 参数
//     robotics::Link links[7] = {
//         robotics::Link(0.0f, 0.34f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.4f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.39f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.126f, 0.0f, 0.0f, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>())
//     };

//     // 初始化机械臂
//     robotics::Serial_Link<7> robot(links);

//     // 初始关节角度（全零）
//     Matrixf<7, 1> q_prev = matrixf::zeros<7, 1>();

//     // 目标位姿
//     float rpy_data[3] = {0.1f, 0.2f, 0.3f}; // [yaw, pitch, roll]
//     float p_data[3] = {0.2f, 0.2f, 0.2f};   // [px, py, pz]
//     Matrixf<3, 1> rpy(rpy_data);
//     Matrixf<3, 1> p(p_data);
//     Matrixf<4, 4> Td = robotics::rp2t(robotics::rpy2r(rpy), p);

//     // 工作空间验证
//     #if CHECK_WORKSPACE
//     std::cout << "目标位置: (" << p[0][0] << ", " << p[1][0] << ", " << p[2][0] << ")\n";
//     std::cout << "目标位置是否在工作空间内: " << (isPointInWorkspace(robot, p) ? "是" : "否") << "\n";
//     std::cout << "机械臂理论工作空间: \n";
//     std::cout << "- Z范围: [0.34, 1.256] 米\n";
//     std::cout << "- 水平半径范围: [0, 0.926] 米\n";
//     #endif

//     // 输出初始位姿
//     Matrixf<4, 4> T_init = robot.fkine(q_prev);
//     Matrixf<3, 1> p_init = robotics::t2p(T_init);
//     Matrixf<3, 1> rpy_init = robotics::t2rpy(T_init);
//     std::cout << "初始位姿:\n";
//     std::cout << "位置: (" << p_init[0][0] << ", " << p_init[1][0] << ", " << p_init[2][0] << ")\n";
//     std::cout << "姿态(RPY): (" << rpy_init[0][0] << ", " << rpy_init[1][0] << ", " << rpy_init[2][0] << ")\n";

//     // 逆运动学求解（调用修正后的 ikine_smooth）
//     float tol = 1e-4f; // 容差
//     uint16_t max_iter = 100; // 最大迭代次数
    
//     // 保存迭代日志
//     #if SAVE_ITERATION_LOG
//     std::ofstream log_file("ik_iteration.log");
//     log_file << "迭代次数,位置误差范数,姿态误差范数,雅可比条件数\n";
//     #endif
    
//     // 修改ikine_smooth函数，增加迭代过程输出
//     Matrixf<7, 1> q = q_prev;
//     bool converged = false;
    
//     std::cout << "\n开始迭代求解...\n";
//     std::cout << "迭代次数 | 位置误差范数 | 姿态误差范数 | 雅可比条件数 | 关节角度变化范数\n";
//     std::cout << "------------------------------------------------------------------\n";
    
//     for (uint16_t iter = 0; iter < max_iter; iter++) {
//         // 计算当前位姿
//         Matrixf<4, 4> T = robot.fkine(q);
        
//         // 计算误差
//         Matrixf<3, 1> p_error = robotics::t2p(Td) - robotics::t2p(T);
//         Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T));
//         Matrixf<3, 1> w_error;
//         for (int i = 0; i < 3; ++i) {
//             w_error[i][0] = twist[i + 3][0]; // 提取姿态误差
//         }
        
//         // 计算雅可比矩阵
//         Matrixf<6, 7> J = robot.jacob(q);
        
//         // 计算雅可比矩阵条件数
//         float cond = conditionNumber(J.trans() * J);
        
//         // 检查收敛
//         if (p_error.norm() < tol && w_error.norm() < tol) {
//             converged = true;
//             break;
//         }
        
//         // 迭代更新（这里简化实现，实际应调用ikine_smooth内部逻辑）
//         Matrixf<7, 1> dq;
        
//         // 阻尼最小二乘法求解 dq
//         Matrixf<7, 7> JTJ = J.trans() * J;
//         float lambda = 0.1f; // 阻尼系数
//         Matrixf<7, 7> damping = lambda * lambda * matrixf::eye<7, 7>();
        
//         // 合并位置和姿态误差，位置误差权重更高
//         Matrixf<6, 1> weighted_err;
//         for (int i = 0; i < 3; i++) weighted_err[i][0] = p_error[i][0] * 10.0f; // 位置误差权重×10
//         for (int i = 3; i < 6; i++) weighted_err[i][0] = w_error[i-3][0];
        
//         dq = matrixf::inv(JTJ + damping) * J.trans() * weighted_err;
        
//         // 限制最大步长
//         float dq_norm = dq.norm();
//         if (dq_norm > 0.1f) { // 最大步长限制
//             dq = dq * (0.1f / dq_norm);
//         }
        
//         // 更新关节角度
//         Matrixf<7, 1> q_next = q + dq;
        
//         // 输出迭代信息
//         #if DEBUG_MODE
//         std::cout << std::setw(4) << iter << "    | " 
//                   << std::setw(12) << p_error.norm() << " | " 
//                   << std::setw(12) << w_error.norm() << " | " 
//                   << std::setw(12) << cond << " | "
//                   << std::setw(14) << dq.norm() << "\n";
//         #endif
        
//         // 保存迭代日志
//         #if SAVE_ITERATION_LOG
//         log_file << iter << "," << p_error.norm() << "," << w_error.norm() << "," << cond << "\n";
//         #endif
        
//         q = q_next;
//     }
    
//     #if SAVE_ITERATION_LOG
//     log_file.close();
//     #endif
    
//     // 验证最终结果
//     Matrixf<4, 4> T_result = robot.fkine(q);
//     Matrixf<3, 1> p_result = robotics::t2p(T_result);
//     Matrixf<3, 1> rpy_result = robotics::t2rpy(T_result);
//     Matrixf<3, 1> p_error = robotics::t2p(Td) - p_result;
//     Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T_result));
//     Matrixf<3, 1> w_error;
//     for (int i = 0; i < 3; ++i) {
//         w_error[i][0] = twist[i + 3][0]; // 提取姿态误差
//     }

//     // 输出最终结果
//     std::cout << "\n逆运动学求解完成!\n";
//     std::cout << std::fixed << std::setprecision(6);
//     std::cout << "目标位姿:\n";
//     std::cout << "位置: (" << p[0][0] << ", " << p[1][0] << ", " << p[2][0] << ")\n";
//     std::cout << "姿态(RPY): (" << rpy[0][0] << ", " << rpy[1][0] << ", " << rpy[2][0] << ")\n";
    
//     std::cout << "\n实际位姿:\n";
//     std::cout << "位置: (" << p_result[0][0] << ", " << p_result[1][0] << ", " << p_result[2][0] << ")\n";
//     std::cout << "姿态(RPY): (" << rpy_result[0][0] << ", " << rpy_result[1][0] << ", " << rpy_result[2][0] << ")\n";
    
//     std::cout << "\n位姿误差:\n";
//     std::cout << "位置误差: (" << p_error[0][0] << ", " << p_error[1][0] << ", " << p_error[2][0] << ")\n";
//     std::cout << "姿态误差: (" << w_error[0][0] << ", " << w_error[1][0] << ", " << w_error[2][0] << ")\n";
    
//     std::cout << "\n逆运动学结果 (关节角度, 单位: rad):\n";
//     for (int i = 0; i < 7; ++i) {
//         std::cout << "q[" << i << "] = " << q[i][0];
//         // 检查关节角度是否在范围内（调用 const 版本的 qmin/qmax）
//         if (q[i][0] < links[i].qmin() || q[i][0] > links[i].qmax()) {
//             std::cout << " (超出范围: [" << links[i].qmin() << ", " << links[i].qmax() << "])";
//         }
//         std::cout << "\n";
//     }
//     std::cout << "位置误差范数: " << p_error.norm() << "\n";
//     std::cout << "姿态误差范数: " << w_error.norm() << "\n";
//     std::cout << "逆运动学是否收敛: " << (converged ? "是" : "否") << "\n";
    
//     // 输出收敛建议
//     if (!converged) {
//         std::cout << "\n收敛建议:\n";
//         if (p_error.norm() > 0.1f) {
//             std::cout << "- 位置误差过大，可能目标点超出工作空间，建议调整目标位置\n";
//         }
//         if (w_error.norm() > 0.1f) {
//             std::cout << "- 姿态误差过大，建议调整目标姿态或增加迭代次数\n";
//         }
//         std::cout << "- 尝试修改阻尼系数(lambda)或步长限制\n";
//         std::cout << "- 检查DH参数是否正确\n";
//     }

//     return 0;
// }

































// // /**
// //  ******************************************************************************
// //  * @file    main.cpp
// //  * @brief   测试 7DOF 机械臂逆运动学 (修复震荡版本)
// //  * @author  Spoon Guan
// //  * @date    2025-07-26
// //  ******************************************************************************
// //  */

// // #include "robotics.h"
// // #include <iostream>
// // #include <iomanip>
// // #include <fstream>
// // #include <cmath>

// // #ifndef PI
// // #define PI 3.141592653589793f
// // #endif

// // // 调试开关
// // #define DEBUG_MODE 1
// // #define SAVE_ITERATION_LOG 0
// // #define CHECK_WORKSPACE 1

// // // 计算矩阵条件数（简化版）
// // template <int R, int C>
// // float conditionNumber(const Matrixf<R, C>& mat) {
// //     float max = 0, min = 1e9;
// //     for (int i = 0; i < std::min(R, C); i++) {
// //         float val = std::abs(mat[i][i]);
// //         if (val > max) max = val;
// //         if (val < min && val > 1e-9) min = val;
// //     }
// //     return max / min;
// // }

// // // 验证点是否在工作空间内（关键修复：目标点必须在工作空间内）
// // bool isPointInWorkspace(const robotics::Serial_Link<7>& robot, const Matrixf<3, 1>& p) {
// //     float min_z = 0.34f;  // 基座高度，目标z必须≥此值
// //     float max_z = 0.34f + 0.4f + 0.39f + 0.126f;  // 最大可达高度≈1.256m
// //     float max_r = 0.4f + 0.4f + 0.126f;  // 最大水平半径≈0.926m
    
// //     float x = p[0][0], y = p[1][0], z = p[2][0];
// //     float r = std::sqrt(x*x + y*y);
    
// //     return (z >= min_z && z <= max_z && r <= max_r);
// // }

// // int main() {
// //     // 1. 定义7DOF机械臂DH参数（保持不变）
// //     robotics::Link links[7] = {
// //         robotics::Link(0.0f, 0.34f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
// //         robotics::Link(0.0f, 0.0f, 0.4f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
// //         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
// //         robotics::Link(0.0f, 0.0f, 0.39f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
// //         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
// //         robotics::Link(0.0f, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
// //         robotics::Link(0.0f, 0.126f, 0.0f, 0.0f, robotics::R, 0.0f, -PI, PI, 1.0f,
// //                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>())
// //     };

// //     robotics::Serial_Link<7> robot(links);
// //     Matrixf<7, 1> q_prev = matrixf::zeros<7, 1>();

// //     // 2. 目标位姿（关键修复：调整z到工作空间内，原0.2m→0.5m）
// //     float rpy_data[3] = {0.1f, 0.2f, 0.3f};  // 姿态不变
// //     float p_data[3] = {0.2f, 0.2f, 0.5f};    // z从0.2→0.5（≥0.34）
// //     Matrixf<3, 1> rpy(rpy_data);
// //     Matrixf<3, 1> p(p_data);
// //     Matrixf<4, 4> Td = robotics::rp2t(robotics::rpy2r(rpy), p);

// //     // 3. 工作空间验证（必须通过，否则无法收敛）
// //     #if CHECK_WORKSPACE
// //     std::cout << "目标位置: (" << p[0][0] << ", " << p[1][0] << ", " << p[2][0] << ")\n";
// //     std::cout << "目标是否在工作空间内: " << (isPointInWorkspace(robot, p) ? "是" : "否") << "\n";
// //     std::cout << "工作空间范围: Z∈[0.34, 1.256]m, 水平半径≤0.926m\n";
// //     if (!isPointInWorkspace(robot, p)) {
// //         std::cout << "错误：目标点超出工作空间，强制退出！\n";
// //         return -1;  // 目标不可达，无需继续迭代
// //     }
// //     #endif

// //     // 4. 逆运动学求解（核心改进：动态参数调整）
// //     float tol = 1e-4f;
// //     uint16_t max_iter = 150;  // 适当增加迭代次数
// //     Matrixf<7, 1> q = q_prev;
// //     bool converged = false;
// //     Matrixf<7, 1> dq_prev = matrixf::zeros<7, 1>();  // 平滑项：保存上一步长

// //     std::cout << "\n开始迭代求解...\n";
// //     std::cout << "迭代次数 | 位置误差范数 | 姿态误差范数 | 雅可比条件数 | 关节角度变化范数\n";
// //     std::cout << "------------------------------------------------------------------\n";
    
// //     for (uint16_t iter = 0; iter < max_iter; iter++) {
// //         // 计算当前位姿和误差
// //         Matrixf<4, 4> T = robot.fkine(q);
// //         Matrixf<3, 1> p_error = robotics::t2p(Td) - robotics::t2p(T);
// //         Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T));
// //         Matrixf<3, 1> w_error;
// //         for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];
// //         Matrixf<6, 7> J = robot.jacob(q);
// //         float cond = conditionNumber(J.trans() * J);

// //         // 检查收敛
// //         if (p_error.norm() < tol && w_error.norm() < tol) {
// //             converged = true;
// //             break;
// //         }

// //         // 5. 动态阻尼系数（误差大→阻尼小，加速收敛；误差小→阻尼大，抑制震荡）
// //         float error_norm = std::sqrt(p_error.norm() + w_error.norm());
// //         float lambda = 0.005f + 0.05f * (1.0f - std::exp(-10.0f * error_norm));  // 动态阻尼

// //         // 6. 动态误差权重（位置误差大时权重高，小时降低）
// //         float pos_weight = std::max(1.0f, 8.0f * std::exp(-8.0f * p_error.norm()));  // 动态位置权重
// //         float att_weight = std::max(1.0f, 5.0f * std::exp(-8.0f * w_error.norm()));  // 动态姿态权重
// //         Matrixf<6, 1> weighted_err;
// //         for (int i = 0; i < 3; i++) weighted_err[i][0] = p_error[i][0] * pos_weight;
// //         for (int i = 3; i < 6; i++) weighted_err[i][0] = w_error[i-3][0] * att_weight;

// //         // 7. 计算关节更新量（阻尼最小二乘）
// //         Matrixf<7, 7> JTJ = J.trans() * J;
// //         Matrixf<7, 7> damping = lambda * lambda * matrixf::eye<7, 7>();
// //         Matrixf<7, 1> dq = matrixf::inv(JTJ + damping) * J.trans() * weighted_err;

// //         // 8. 动态步长限制（误差大→步长大，误差小→步长小，避免超调）
// //         float dq_norm = dq.norm();
// //         float max_step = 0.08f * std::min(1.0f, std::max(p_error.norm(), w_error.norm()) * 15.0f);  // 动态步长
// //         if (dq_norm > max_step) dq = dq * (max_step / dq_norm);

// //         // 9. 平滑项：融合上一步长，抑制震荡（关键修复）
// //         float smooth_factor = 0.2f;  // 平滑系数（0.1~0.3）
// //         dq = (1 - smooth_factor) * dq + smooth_factor * dq_prev;
// //         dq_prev = dq;  // 更新平滑项

// //         // 更新关节角度
// //         q = q + dq;

// //         // 输出迭代信息
// //         #if DEBUG_MODE
// //         std::cout << std::setw(4) << iter << "    | " 
// //                   << std::setw(12) << p_error.norm() << " | " 
// //                   << std::setw(12) << w_error.norm() << " | " 
// //                   << std::setw(12) << cond << " | "
// //                   << std::setw(14) << dq.norm() << "\n";
// //         #endif
// //     }

// //     // 10. 结果验证与输出
// //     Matrixf<4, 4> T_result = robot.fkine(q);
// //     Matrixf<3, 1> p_result = robotics::t2p(T_result);
// //     Matrixf<3, 1> p_error = robotics::t2p(Td) - p_result;
// //     Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T_result));
// //     Matrixf<3, 1> w_error;
// //     for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];

// //     std::cout << "\n最终结果:\n";
// //     std::cout << "位置误差范数: " << p_error.norm() << " (目标<" << tol << ")\n";
// //     std::cout << "姿态误差范数: " << w_error.norm() << " (目标<" << tol << ")\n";
// //     std::cout << "是否收敛: " << (converged ? "是" : "否") << "\n";

// //     return 0;
// // }










































// /**
//  ******************************************************************************
//  * @file    main.cpp
//  * @brief   测试 7DOF 机械臂逆运动学 (零空间优化+奇异鲁棒版)
//  * @author  Spoon Guan
//  * @date    2025-07-26
//  ******************************************************************************
//  */

// #include "robotics.h"
// #include <iostream>
// #include <iomanip>
// #include <fstream>
// #include <cmath>

// #ifndef PI
// #define PI 3.141592653589793f
// #endif

// // 调试开关
// #define DEBUG_MODE 1
// #define SAVE_ITERATION_LOG 0
// #define CHECK_WORKSPACE 1
// #define AVOID_SINGULARITIES 1  // 启用奇异解规避

// // 计算矩阵条件数（用于监测雅可比矩阵奇异性）
// template <int R, int C>
// float conditionNumber(const Matrixf<R, C>& mat) {
//     float max = 0, min = 1e9;
//     for (int i = 0; i < std::min(R, C); i++) {
//         float val = std::abs(mat[i][i]);
//         if (val > max) max = val;
//         if (val < min && val > 1e-9) min = val;
//     }
//     return max / min;
// }

// // 验证点是否在工作空间内
// bool isPointInWorkspace(const robotics::Serial_Link<7>& robot, const Matrixf<3, 1>& p) {
//     float min_z = 0.34f;  // 基座高度
//     float max_z = 0.34f + 0.4f + 0.39f + 0.126f;  // 最大可达高度
//     float max_r = 0.4f + 0.4f + 0.126f;  // 最大水平半径
    
//     float x = p[0][0], y = p[1][0], z = p[2][0];
//     float r = std::sqrt(x*x + y*y);
    
//     return (z >= min_z && z <= max_z && r <= max_r);
// }

// // 计算雅可比矩阵的伪逆和零空间投影矩阵
// template <int R, int C>
// struct JacobianInfo {
//     Matrixf<C, R> pinv;      // 伪逆
//     Matrixf<C, C> null_proj; // 零空间投影矩阵
//     float cond;             // 条件数
//     float min_sv;           // 最小奇异值
// };

// template <int R, int C>
// JacobianInfo<R, C> computeJacobianInfo( Matrixf<R, C>& J, float lambda) {
//     JacobianInfo<R, C> info;
    
//     // 计算雅可比矩阵条件数（简化版）
//     info.cond = conditionNumber(J.trans() * J);
    
//     // 计算阻尼最小二乘伪逆 J† = (JᵀJ + λ²I)⁻¹Jᵀ
//     Matrixf<C, C> JTJ = J.trans() * J;
//     Matrixf<C, C> damping = lambda * lambda * matrixf::eye<C, C>();
//     info.pinv = matrixf::inv(JTJ + damping) * J.trans();
    
//     // 计算零空间投影矩阵 P = I - J†J
//     info.null_proj = matrixf::eye<C, C>() - info.pinv * J;
    
//     // 估计最小奇异值（用于奇异检测）
//     // 注意：此处为简化实现，实际应用中应使用SVD分解
//     info.min_sv = 1.0f / std::sqrt(info.cond);
    
//     return info;
// }

// int main() {
//     // 定义7DOF机械臂DH参数
//     robotics::Link links[7] = {
//         robotics::Link(0.0f, 0.34f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.4f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.39f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.4f, 0.0f, PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.0f, 0.0f, -PI / 2, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>()),
//         robotics::Link(0.0f, 0.126f, 0.0f, 0.0f, robotics::R, 0.0f, -PI, PI, 1.0f,
//                        matrixf::zeros<3, 1>(), matrixf::zeros<3, 3>())
//     };

//     robotics::Serial_Link<7> robot(links);
//     Matrixf<7, 1> q_prev = matrixf::zeros<7, 1>();

//     // 目标位姿（确保在工作空间内）
//     float rpy_data[3] = {0.1f, 0.2f, 0.3f};  // RPY姿态
//     float p_data[3] = {0.2f, 0.2f, 0.5f};    // 位置（z≥0.34）
//     Matrixf<3, 1> rpy(rpy_data);
//     Matrixf<3, 1> p(p_data);
//     Matrixf<4, 4> Td = robotics::rp2t(robotics::rpy2r(rpy), p);

//     // 工作空间验证
//     #if CHECK_WORKSPACE
//     std::cout << "目标位置: (" << p[0][0] << ", " << p[1][0] << ", " << p[2][0] << ")\n";
//     if (!isPointInWorkspace(robot, p)) {
//         std::cout << "错误：目标点超出工作空间！\n";
//         return -1;
//     }
//     #endif

//     // 逆运动学求解
//     float tol = 1e-4f;
//     uint16_t max_iter = 150;
//     Matrixf<7, 1> q = q_prev;
//     bool converged = false;
//     Matrixf<7, 1> dq_prev = matrixf::zeros<7, 1>();

//     std::cout << "\n开始迭代求解...\n";
//     std::cout << "迭代次数 | 位置误差范数 | 姿态误差范数 | 雅可比条件数 | 关节角度变化范数\n";
//     std::cout << "------------------------------------------------------------------\n";
    
//     for (uint16_t iter = 0; iter < max_iter; iter++) {
//         // 计算当前位姿和误差
//         Matrixf<4, 4> T = robot.fkine(q);
//         Matrixf<3, 1> p_error = robotics::t2p(Td) - robotics::t2p(T);
//         Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T));
//         Matrixf<3, 1> w_error;
//         for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];
//         Matrixf<6, 7> J = robot.jacob(q);

//         // 检查收敛
//         if (p_error.norm() < tol && w_error.norm() < tol) {
//             converged = true;
//             break;
//         }

//         // 1. 动态阻尼系数（基于误差和奇异值）
//         float error_norm = std::sqrt(p_error.norm() + w_error.norm());
//         float base_lambda = 0.005f + 0.05f * (1.0f - std::exp(-10.0f * error_norm));
        
//         // 2. 奇异值检测与增强阻尼
//         float lambda = base_lambda;
//         #if AVOID_SINGULARITIES
//         JacobianInfo<6, 7> jac_info = computeJacobianInfo(J, base_lambda);
//         float cond = jac_info.cond;
//         float min_sv = jac_info.min_sv;
        
//         // 接近奇异位形时增大阻尼
//         if (min_sv < 0.05f) {
//             lambda = base_lambda + 0.5f * (0.05f - min_sv);  // 奇异时阻尼增大
//             std::cout << "警告：接近奇异位形！条件数=" << cond << ", 阻尼=" << lambda << "\n";
//         } else {
//             cond = conditionNumber(J.trans() * J);
//         }
//         #else
//         float cond = conditionNumber(J.trans() * J);
//         #endif

//         // 3. 动态误差权重
//         float pos_weight = std::max(1.0f, 8.0f * std::exp(-8.0f * p_error.norm()));
//         float att_weight = std::max(1.0f, 5.0f * std::exp(-8.0f * w_error.norm()));
//         Matrixf<6, 1> weighted_err;
//         for (int i = 0; i < 3; i++) weighted_err[i][0] = p_error[i][0] * pos_weight;
//         for (int i = 3; i < 6; i++) weighted_err[i][0] = w_error[i-3][0] * att_weight;

//         // 4. 零空间优化目标：保持关节接近上一位置 & 远离奇异
//         Matrixf<7, 1> primary_task = matrixf::zeros<7, 1>();
//         Matrixf<7, 1> secondary_task = matrixf::zeros<7, 1>();
        
//         // 主任务：末端执行器位姿跟踪
//         primary_task = computeJacobianInfo(J, lambda).pinv * weighted_err;
        
//         // 次任务：关节限位与奇异规避
//         #if AVOID_SINGULARITIES
//         // a. 关节限位优化：向关节范围中心移动
//         for (int i = 0; i < 7; i++) {
//             float q_center = (links[i].qmin() + links[i].qmax()) / 2.0f;
//             secondary_task[i][0] = 0.5f * (q_center - q[i][0]);  // 向中心吸引
//         }
        
//         // b. 奇异规避（基于雅可比行列式梯度）
//         // 简化实现：接近奇异时，沿梯度方向远离
//         if (min_sv < 0.05f) {
//             // 计算雅可比行列式对各关节的梯度（简化版）
//             for (int i = 0; i < 7; i++) {
//                 Matrixf<7, 1> q_perturbed = q;
//                 q_perturbed[i][0] += 0.001f;  // 微小扰动
//                 Matrixf<6, 7> J_perturbed = robot.jacob(q_perturbed);
//                 float cond_perturbed = conditionNumber(J_perturbed.trans() * J_perturbed);
                
//                 // 沿减小条件数的方向移动
//                 secondary_task[i][0] += 0.1f * (cond - cond_perturbed);
//             }
//         }
//         #else
//         // 仅保持关节接近上一位置
//         secondary_task = -2.0f * (q - q_prev);
//         #endif

//         // 5. 零空间投影：将次任务投影到雅可比零空间
//         Matrixf<7, 1> null_proj = matrixf::zeros<7, 1>();
//         #if AVOID_SINGULARITIES
//         null_proj = jac_info.null_proj * secondary_task;
//         #else
//         // 如果未计算零空间投影矩阵，则简化计算
//         Matrixf<7, 7> null_space = matrixf::eye<7, 7>() - computeJacobianInfo(J, lambda).pinv * J;
//         null_proj = null_space * secondary_task;
//         #endif

//         // 6. 合并主任务和零空间优化
//         Matrixf<7, 1> dq = primary_task + null_proj;

//         // 7. 动态步长限制
//         float dq_norm = dq.norm();
//         float max_step = 0.08f * std::min(1.0f, std::max(p_error.norm(), w_error.norm()) * 15.0f);
//         if (dq_norm > max_step) dq = dq * (max_step / dq_norm);

//         // 8. 平滑项
//         float smooth_factor = 0.2f;
//         dq = (1 - smooth_factor) * dq + smooth_factor * dq_prev;
//         dq_prev = dq;

//         // 更新关节角度
//         q = q + dq;

//         // 输出迭代信息
//         #if DEBUG_MODE
//         std::cout << std::setw(4) << iter << "    | " 
//                   << std::setw(12) << p_error.norm() << " | " 
//                   << std::setw(12) << w_error.norm() << " | " 
//                   << std::setw(12) << cond << " | "
//                   << std::setw(14) << dq.norm() << "\n";
//         #endif
//     }

//     // 结果验证与输出
//     Matrixf<4, 4> T_result = robot.fkine(q);
//     Matrixf<3, 1> p_result = robotics::t2p(T_result);
//     Matrixf<3, 1> p_error = robotics::t2p(Td) - p_result;
//     Matrixf<6, 1> twist = robotics::t2twist(Td * robotics::invT(T_result));
//     Matrixf<3, 1> w_error;
//     for (int i = 0; i < 3; ++i) w_error[i][0] = twist[i + 3][0];

//     std::cout << "\n最终结果:\n";
//     std::cout << "位置误差范数: " << p_error.norm() << " (目标<" << tol << ")\n";
//     std::cout << "姿态误差范数: " << w_error.norm() << " (目标<" << tol << ")\n";
//     std::cout << "是否收敛: " << (converged ? "是" : "否") << "\n";

//     return 0;
// }











































