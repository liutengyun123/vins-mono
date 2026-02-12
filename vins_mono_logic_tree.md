# VINS-Mono 后端完整代码逻辑树状图

本文档严格按照代码执行流程 (`estimator_node.cpp` -> `estimator.cpp`) 整理，展示了 VINS-Mono 后端核心线程的完整调用层级、输入参数及关键功能。

---

## 根节点: `process()` 线程
**位置**: `vins_estimator/src/estimator_node.cpp`
**功能**: 后端主循环，持续运行。

### 1 `getMeasurements()`
*   **输入**: 无 (读取全局缓冲队列 `imu_buf`, `feature_buf`)
*   **功能**: 阻塞等待并获取**一对齐**的数据包。确保 IMU 数据的时间覆盖了图像帧的时间段 (考虑 `td`)。
*   **输出**: `std::vector<std::pair<std::vector<ImuConstPtr>, PointCloudConstPtr>> measurements`

### 2 `estimator.processIMU(...)`
*   **输入**: 
    *   `dt`: double (时间间隔)
    *   `linear_acceleration`: Vector3d (线加速度)
    *   `angular_velocity`: Vector3d (角速度)
*   **功能**: 处理 IMU 数据，进行预积分和状态递推。
*   **内部调用**:
    *   **2.1 `IntegrationBase::push_back(...)`**
        *   **输入**: `dt`, `acc`, `gyr`
        *   **功能**: **中值积分 (Mid-point Integration)**。更新预积分量 ($\alpha, \beta, \gamma$)，更新协方差矩阵 $P$ 和 雅可比矩阵 $J$。
    *   **2.2 `Rs[j] *= Utility::deltaQ(...)`**
        *   **功能**: 姿态递推 (四元数乘法)。
    *   **2.3 `Ps[j] += ...`, `Vs[j] += ...`**
        *   **功能**: 位置和速度递推 (利用牛顿运动定律: $s = vt + 0.5at^2$)。

### 3 `estimator.setReloFrame(...)` (可选)
*   **触发条件**: `relo_buf` 不为空 (收到回环检测消息)。
*   **输入**: 
    *   `_frame_stamp`: double
    *   `_frame_index`: int
    *   `_match_points`: vector<Vector3d>
    *   `_relo_t`: Vector3d
    *   `_relo_r`: Matrix3d
*   **功能**: 将回环帧的信息注册到 Estimator 中，用于后续的全局优化。

### 4 `estimator.processImage(...)` (核心)
*   **输入**:
    *   `image`: `map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>` (特征点数据)
    *   `header`: `std_msgs::Header` (图像时间戳)
*   **功能**: VINS 状态机核心。处理特征点，决定关键帧，执行初始化或非线性优化，滑动窗口。
*   **内部调用**:
    *   **4.1 `f_manager.addFeatureCheckParallax(...)`**
        *   **输入**: `frame_count`, `image`, `td`
        *   **功能**: 检查新帧与上一关键帧的视差。
        *   **输出**: 设置 `marginalization_flag` (`MARGIN_OLD` 或 `MARGIN_SECOND_NEW`)。

    *   **4.2 `if (solver_flag == INITIAL)` (系统初始化)**
        *   **4.2.1 `initialStructure()`**
            *   **功能**: 视觉惯性联合初始化。
            *   **内部调用**:
                *   **`check_imu_observability`**: 计算加速度方差，判断是否静止。
                *   **`f_manager.getCorresponding(...)`**: 寻找有足够视差的两帧。
                *   **`m_estimator.solveRelativeRT(...)`**: 恢复参考帧之间的相对 R, T。
                *   **`GlobalSFM::construct(...)`**: 纯视觉 SfM，恢复滑动窗口内所有帧的位姿。
                *   **`cv::solvePnP(...)`**: 将剩余帧对齐到 SfM 做标系。
                *   **`visualInitialAlign()`**: **视觉-IMU 对齐**。
                    *   `VisualIMUAlignment(...)`: 求解尺度 $s$、重力 $g$、速度 $V$、陀螺仪偏置 $B_g$。
            *   **成功后**: `solver_flag = NON_LINEAR`，调用 `solveOdometry()`，然后 `slideWindow()`。

    *   **4.3 `if (solver_flag == NON_LINEAR)` (VIO 正常紧耦合优化)**
        *   **4.3.1 `solveOdometry()`**
            *   **功能**: 执行一次完整的 VIO 优化。
            *   **内部调用**:
                *   **`f_manager.triangulate(...)`**: 三角化新特征点，计算初始深度。
                *   **`optimization()`** (**核心优化函数**)
                    *   **步骤 1: 构建问题**
                        *   `ceres::Problem problem`
                        *   `AddParameterBlock`: 
                            *   `para_Pose`: 7维 (P, Q)
                            *   `para_SpeedBias`: 9维 (V, Ba, Bg)
                            *   `para_Ex_Pose`: 外参 (TIC, RIC)
                            *   `para_Td`: 时间偏移
                            *   `para_Feature`: 特征点逆深度
                    *   **步骤 2: 添加残差 (Factors)**
                        *   **`MarginalizationFactor`**: 添加边缘化先验残差 (来自 `last_marginalization_info`)。
                        *   **`IMUFactor`**: 添加 IMU 预积分残差 (约束相邻帧 PVQ, Bias)。
                        *   **`ProjectionFactor`**: 添加视觉重投影误差。
                        *   **`ProjectionTdFactor`**: 添加带时间偏移的视觉误差 (若 `ESTIMATE_TD` 开启)。
                    *   **步骤 3: 求解**
                        *   `ceres::Solve(options, &problem, &summary)`
                    *   **步骤 4: 更新状态**
                        *   `double2vector()`: 将优化后的 double 数组拷回 Eigen 状态变量。
        
        *   **4.3.2 `failureDetection()`**
            *   **功能**: 检测系统是否发散 (如 Bias 过大, 轨迹跳变)。
        
        *   **4.3.3 `slideWindow()`**
            *   **功能**: 滑动窗口，执行边缘化。
            *   **内部调用**:
                *   **`slideWindowOld()`** (若 `MARGIN_OLD`):
                    *   移除最老帧 (Frame 0)。
                    *   构建 `MarginalizationInfo` (Schur 补)，生成新的先验。
                *   **`slideWindowNew()`** (若 `MARGIN_SECOND_NEW`):
                    *   移除次新帧。直接丢弃视觉观测，合并 IMU 预积分。
        
        *   **4.3.4 `f_manager.removeFailures()`**
            *   **功能**: 移除深度为负或不可观测的特征点。

### 5 结果发布 (Visualization & ROS)

*   **5.1 `pubOdometry(estimator, header)`**
    *   **输入**: `estimator` 所有的状态, `header`
    *   **功能**: 发布 `/vins_estimator/odometry` (nav_msgs::Odometry)。这是**最高精度**的 VIO 输出 (包含 P, Q, V)。

*   **5.2 `pubKeyPoses(estimator, header)`**
    *   **功能**: 发布 `/vins_estimator/key_poses` (sensor_msgs::PointCloud)。
    *   **内容**: 滑动窗口内所有关键帧的位置。

*   **5.3 `pubCameraPose(estimator, header)`**
    *   **功能**: 发布 `/vins_estimator/camera_pose` (nav_msgs::Odometry)。
    *   **内容**: 当前相机的世界坐标系位姿 (不同于 Body 位姿，差一个外参 $T_{bc}$)。

*   **5.4 `pubPointCloud(estimator, header)`**
    *   **功能**: 发布 `/vins_estimator/point_cloud` (sensor_msgs::PointCloud)。
    *   **内容**: 所有有效特征点的 3D 坐标。

*   **5.5 `pubTF(estimator, header)`**
    *   **功能**: 发布 TF 变换 (`world` -> `body`)。

*   **5.6 `pubKeyframe(estimator)`**
    *   **功能**: 发布关键帧数据给闭环检测模块 (`pose_graph`)。
    *   **内容**: 图像数据、位姿、特征点。

*   **5.7 `update()`** (在 `process()` 循环末尾)
    *   **功能**: 更新 IMU 预测器的基准状态 (acc_0, gyr_0, P, Q, V, Ba, Bg)，用于下一次 `imu_callback` 中的高频递推。
