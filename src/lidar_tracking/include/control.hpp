/**
 * @file control.hpp
 * @brief PID控制器头文件
 * @details 实现位置式和增量式PID控制器，用于车辆路径跟踪控制
 * @author zyh
 * @version 1.0
 * @date 2025-08-24
 */

namespace Control
{
    /**
     * @brief PID控制器类
     * @details 提供位置式和增量式两种PID控制算法
     * 位置式PID：输出为绝对控制量，适合大多数控制场景
     * 增量式PID：输出为控制量增量，适合执行器有积分特性的场景
     */
    class PID 
    {
    public:
        // ===== PID控制参数 =====
        double kp;                    // 比例系数：控制响应速度和稳态精度
        double ki;                    // 积分系数：消除稳态误差，提高控制精度
        double kd;                    // 微分系数：改善动态特性，提高系统稳定性
        
        // ===== 控制器状态变量 =====
        double error_out;             // 控制器输出：当前时刻的控制量
        double last_error;            // 上一次误差：用于计算误差变化率
        double integral;              // 积分项：误差的累积和
        double inte_max;              // 积分限幅：防止积分饱和，限制积分项的最大值
        double last_diff;             // 上一次微分值：用于增量式PID计算
        
        // ===== 控制算法接口 =====
        /**
         * @brief 位置式PID控制器
         * @param error 当前误差输入
         * @return 绝对控制量输出
         */
        double PIDPositional(double error);
        
        /**
         * @brief 增量式PID控制器
         * @param error 当前误差输入
         * @return 控制量增量输出
         */
        double PIDIncremental(double error);
        
        /**
         * @brief PID控制器初始化
         * @details 设置初始参数值，清零状态变量
         */
        void Init();
        
    };
    
    // ===== 位置式PID控制器实现 =====
    /**
     * @brief 位置式PID控制器实现
     * @details 输出为绝对控制量，计算公式：u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
     * 优点：输出稳定，适合大多数控制场景
     * 缺点：积分饱和可能导致超调
     * @param error 当前时刻的误差输入
     * @return 绝对控制量输出
     */
    double PID::PIDPositional(double error)
    {
        // 积分项累加：累积误差
        integral += error;

        // 积分限幅：防止积分饱和，限制积分项的最大值
        if(integral > inte_max)
            integral = inte_max;

        // PID控制算法：比例项 + 积分项 + 微分项
        error_out = (kp * error) +                    // 比例项：Kp * e(t)
                   (ki * integral) +                   // 积分项：Ki * ∫e(t)dt
                   (kd * (error - last_error));        // 微分项：Kd * de(t)/dt
        
        // 更新上一次误差：为下一时刻的微分计算做准备
        last_error = error;
        
        // 返回控制量输出
        return error_out;
    }
    
    // ===== 增量式PID控制器实现 =====
    /**
     * @brief 增量式PID控制器实现
     * @details 输出为控制量增量，计算公式：Δu(t) = Kp*Δe(t) + Ki*e(t) + Kd*Δ²e(t)
     * 优点：积分饱和影响小，适合执行器有积分特性的场景
     * 缺点：输出可能不够稳定
     * @param error 当前时刻的误差输入
     * @return 控制量增量输出
     */
    double PID::PIDIncremental(double error)
    {
        // 增量式PID控制算法：比例增量 + 积分项 + 二阶微分项
        error_out = kp * (error - last_error) +                    // 比例增量项：Kp * Δe(t)
                   ki * error +                                     // 积分项：Ki * e(t)
                   kd * ((error - last_error) - last_diff);        // 二阶微分项：Kd * Δ²e(t)

        // 更新上一次微分值：为下一时刻的二阶微分计算做准备
        last_diff = error - last_error;
        
        // 更新上一次误差：为下一时刻的增量计算做准备
        last_error = error;

        // 返回控制量增量输出
        return error_out;
    }

    // ===== PID控制器初始化函数 =====
    /**
     * @brief PID控制器初始化
     * @details 设置初始参数值，清零所有状态变量
     * 注意：这些参数值需要根据实际控制对象进行调整
     */
    void PID::Init()
    {
        // ===== PID控制参数设置 =====
        kp = 0.88;                  // 比例系数：控制响应速度，值越大响应越快但可能超调
        ki = 0.0;                   // 积分系数：消除稳态误差，当前设为0（未使用积分控制）
        kd = 1.2;                   // 微分系数：改善动态特性，提高系统稳定性
        
        // ===== 状态变量初始化 =====
        error_out = 0.0;            // 控制器输出：初始化为0
        last_error = 0.0;           // 上一次误差：初始化为0
        integral = 0.0;             // 积分项：初始化为0
        inte_max = 8.0;             // 积分限幅：设置为8.0，防止积分饱和
        
        // ===== 增量式PID专用变量初始化 =====
        last_diff = 0.0;            // 上一次微分值：初始化为0
        
        // ===== 备用参数配置（已注释） =====
        // 以下是一组备用的PID参数，可以根据不同场景切换使用
        // kp = 18.0;               // 备用比例系数：适用于需要快速响应的场景
        // ki = 0.1;                // 备用积分系数：适用于需要消除稳态误差的场景
        // kd = 3.0;                // 备用微分系数：适用于需要高稳定性的场景
        // error_out = 0.0;         // 备用输出初始化
        // last_error = 0.0;        // 备用误差初始化
        // integral = 0.0;          // 备用积分项初始化
        // inte_max = 8.0;          // 备用积分限幅
        // last_diff = 0.0;         // 备用微分值初始化
    }
    
} // namespace Control