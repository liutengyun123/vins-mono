# VINS-Mono 项目全面消化文档

> **VINS-Mono**: A Robust and Versatile Monocular Visual-Inertial State Estimator  
> 来自 **香港科技大学** 空中机器人组 (HKUST Aerial Robotics Group)

---

## 一、项目总览

VINS-Mono 是一个实时的 **单目视觉惯性 SLAM 系统**，基于 **优化的滑动窗口** 公式，提供高精度视觉惯性里程计(VIO)。

### 核心特性
| 特性                 | 说明                                        |
| -------------------- | ------------------------------------------- |
| IMU 预积分           | 用中点积分法进行高效预积分，带偏差校正      |
| 自动初始化           | SfM + 视觉-IMU 对齐，自动恢复尺度和重力方向 |
| 在线外参标定         | 可在线估计/优化相机-IMU外参 (旋转+平移)     |
| 在线时间标定         | 估计相机和IMU之间的时间偏移 td              |
| 回环检测             | 基于 DBoW2 词袋模型 + BRIEF 描述子          |
| 位姿图优化           | 4-DOF (yaw + xyz) 全局位姿图优化，消除漂移  |
| 地图合并/复用        | 支持多序列地图合并、位姿图保存和加载复用    |
| 失败检测与恢复       | 自动检测估计器发散并重启                    |
| Rolling Shutter 支持 | 支持卷帘快门相机                            |

### 依赖
- **ROS** (Kinetic/Melodic/Noetic)
- **Ceres Solver** (非线性优化)
- **OpenCV** (图像处理)
- **Eigen3** (矩阵运算)
- **DBoW2** (回环检测词袋)

---

## 二、系统架构 — 三个 ROS 节点

系统由 **三个独立的 ROS 节点** 组成，通过 ROS Topic 通信：

```
┌──────────────────────────────────────────────────────────────────┐
│                        euroc.launch                              │
│                                                                  │
│  ┌───────────────┐    ┌─────────────────┐    ┌──────────────┐   │
│  │ feature_tracker│───▶│ vins_estimator  │───▶│  pose_graph  │   │
│  │   (前端)       │    │   (后端/核心)    │    │  (回环闭合)   │   │
│  └───────┬───────┘    └────────┬────────┘    └──────────────┘   │
│          │                     │                                 │
│    /cam0/image_raw       /imu0 (外部)                            │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 三、模块一：feature_tracker (前端特征追踪)

### 3.1 核心文件
| 文件                       | 作用                                   |
| -------------------------- | -------------------------------------- |
| `feature_tracker_node.cpp` | **ROS 节点入口**，订阅图像、发布特征点 |
| `feature_tracker.h/cpp`    | **FeatureTracker 类**，核心跟踪逻辑    |
| `parameters.h/cpp`         | 读取配置参数                           |
| `tic_toc.h`                | 计时工具                               |

### 3.2 数据流

```
图像 (/cam0/image_raw)
    │
    ▼
img_callback()
    │
    ├── 1. 相机流中断检测 (dt > 1s → 重置)
    ├── 2. 频率控制 (控制发布频率 ≤ FREQ Hz)
    ├── 3. 转灰度图 (MONO8)
    │
    ▼
FeatureTracker::readImage()
    │
    ├── 4. 自适应直方图均衡 (CLAHE, 若 EQUALIZE=1)
    ├── 5. LK 光流追踪 (cv::calcOpticalFlowPyrLK)
    │     窗口 21×21, 金字塔 3 层
    ├── 6. 边界检测 (去掉出边界的点)
    ├── 7. 基础矩阵 RANSAC 剔除外点 (rejectWithF)
    │     将像素坐标去畸变到虚拟相机 (f=460), 再用 F 矩阵检验
    ├── 8. 设置 Mask (setMask)
    │     按跟踪时长排序，保留长期跟踪点
    │     在特征点周围画圆形屏蔽区 (MIN_DIST)
    ├── 9. 检测新特征 (cv::goodFeaturesToTrack, Shi-Tomasi 角点)
    │     补充到 MAX_CNT 个特征
    ├── 10. 去畸变 (undistortedPoints)
    │      通过相机模型 liftProjective() 得到归一化坐标
    └── 11. 计算光流速度 (pts_velocity)
           用于 IMU-相机时间偏移补偿

    │
    ▼
发布数据:
  - /feature_tracker/feature  (sensor_msgs::PointCloud)
      ├── points: 归一化坐标 (x, y, 1)
      └── channels: [feature_id, pixel_u, pixel_v, velocity_x, velocity_y]
  - /feature_tracker/feature_img  (可视化图像)
  - /feature_tracker/restart  (重启信号)
```

### 3.3 关键参数
| 参数          | 默认值 | 说明                 |
| ------------- | ------ | -------------------- |
| `max_cnt`     | 150    | 最大特征数量         |
| `min_dist`    | 30     | 特征最小间距 (pixel) |
| `freq`        | 10     | 发布频率 (Hz)        |
| `F_threshold` | 1.0    | RANSAC 阈值 (pixel)  |
| `equalize`    | 1      | 自适应直方图均衡     |
| `fisheye`     | 0      | 鱼眼模式             |

---

## 四、模块二：vins_estimator (后端状态估计 — 核心)

### 4.1 核心文件
| 文件                                       | 作用                                        |
| ------------------------------------------ | ------------------------------------------- |
| `estimator_node.cpp`                       | **ROS 节点入口**，多线程架构                |
| `estimator.h/cpp`                          | **Estimator 类**，核心状态估计 (1129行代码) |
| `feature_manager.h/cpp`                    | **FeatureManager 类**，特征管理与三角化     |
| `parameters.h/cpp`                         | 配置参数管理                                |
| `factor/imu_factor.h`                      | IMU 因子 (Ceres CostFunction)               |
| `factor/integration_base.h`                | **IMU 预积分** (核心算法, 446行)            |
| `factor/projection_factor.h/cpp`           | 视觉重投影因子                              |
| `factor/projection_td_factor.h/cpp`        | 带时间偏移的重投影因子                      |
| `factor/marginalization_factor.h/cpp`      | 边缘化因子                                  |
| `factor/pose_local_parameterization.h/cpp` | SE(3) 位姿参数化                            |
| `initial/initial_alignment.h/cpp`          | **视觉-IMU 对齐** (求解尺度/重力/速度)      |
| `initial/initial_sfm.h/cpp`                | **Structure from Motion**                   |
| `initial/initial_ex_rotation.h/cpp`        | 在线外参旋转标定                            |
| `initial/solve_5pts.h/cpp`                 | 五点法求本质矩阵                            |

### 4.2 多线程架构

```
main()
├── ROS 回调线程 (ros::spin)
│   ├── imu_callback()       → imu_buf (queue)
│   ├── feature_callback()   → feature_buf (queue)
│   ├── restart_callback()   → 重置估计器
│   └── relocalization_callback() → relo_buf
│
└── 处理线程 (std::thread process)
    └── process()
        ├── getMeasurements()  → 时间对齐 IMU 和图像
        ├── processIMU()       → IMU 预积分
        ├── processImage()     → 核心处理
        └── 发布结果
```

### 4.3 IMU-图像时间对齐 (getMeasurements)

```
getMeasurements() 的核心逻辑：

时间轴:  ─────────────────────────────────────────▶
IMU:     |i1|i2|i3|i4|i5|i6|i7|i8|i9|...
Image:          |img1|        |img2|
                     ▲
              img_time + td

输出: vector<pair<vector<IMU>, Image>>
- 每个 Image 配上 "之前的所有 IMU" + "之后的第一个 IMU"
- 最后一个 IMU 做线性插值到图像时间
- 通过 td 补偿 IMU-相机时间偏移
```

### 4.4 状态估计器核心流程 (Estimator)

```
processImage()
│
├── addFeatureCheckParallax()  → 判断关键帧/非关键帧
│   ├── 是关键帧 → marginalization_flag = MARGIN_OLD (边缘化最老帧)
│   └── 非关键帧 → marginalization_flag = MARGIN_SECOND_NEW (边缘化次新帧)
│
├── 在线外参标定 (若 ESTIMATE_EXTRINSIC == 2)
│   └── CalibrationExRotation() → 用旋转约束标定 IMU-相机旋转
│
├── [INITIAL 阶段] (solver_flag == INITIAL)
│   └── initialStructure()
│       ├── 检查 IMU 可观测性 (加速度方差 > 0.25)
│       ├── 构建 SFM 特征
│       ├── relativePose() → 找到有足够视差的帧对
│       ├── GlobalSFM::construct() → 全局 SfM 重建
│       ├── solvePnP() → 对所有帧求解位姿
│       └── visualInitialAlign()
│           ├── VisualIMUAlignment() → 求解尺度 s、重力 g、速度 V、陀螺仪偏差 Bg
│           ├── 应用尺度到位置和深度
│           ├── 对齐重力方向 → 世界坐标系 z 轴对齐重力
│           └── 成功 → solver_flag = NON_LINEAR
│
├── [NON_LINEAR 阶段] (正常运行)
│   ├── solveOdometry()
│   │   ├── triangulate() → 三角化新特征点
│   │   └── optimization() → **Ceres 非线性优化** ⭐
│   │
│   ├── failureDetection() → 失败检测
│   │   ├── 加速度偏差 > 2.5 → 失败
│   │   ├── 陀螺仪偏差 > 1.0 → 失败
│   │   ├── 位移 > 5m → 失败
│   │   └── z 方向位移 > 1m → 失败
│   │
│   └── slideWindow() → 滑动窗口
│       ├── MARGIN_OLD: 移出最老帧, 深度转移
│       └── MARGIN_SECOND_NEW: 移出次新帧, 合并预积分
│
└── 发布结果
```

### 4.5 滑动窗口状态变量

```
WINDOW_SIZE = 10 (保持 11 帧)

对于窗口中的每一帧 i (0 ≤ i ≤ 10):
  Ps[i]  : Vector3d   位置 (3)
  Rs[i]  : Matrix3d   旋转 (3×3)
  Vs[i]  : Vector3d   速度 (3)
  Bas[i] : Vector3d   加速度计偏差 (3)
  Bgs[i] : Vector3d   陀螺仪偏差 (3)

其他状态:
  tic[0] : Vector3d   IMU-相机平移外参 (3)
  ric[0] : Matrix3d   IMU-相机旋转外参 (3×3)
  td     : double     时间偏移 (1)

特征深度:
  para_Feature[NUM_OF_F][1]  最多 1000 个特征的逆深度
```

### 4.6 Ceres 优化 (optimization) ⭐⭐⭐

这是系统最核心的部分，构建并求解一个大型非线性最小二乘问题：

```
优化变量总览:
┌─────────────────────────────────────────────────────┐
│ 11 × Pose     (位置 + 四元数)     = 11 × 7  = 77   │
│ 11 × SpeedBias (速度+加速度偏差   = 11 × 9  = 99   │
│       +陀螺仪偏差)                                   │
│  1 × ExPose   (外参)              = 1 × 7   = 7    │
│  N × Feature  (特征逆深度)         = N × 1          │
│  1 × Td       (时间偏移)           = 1              │
└─────────────────────────────────────────────────────┘

残差 (约束) 类型:
┌─────────────────────────────────────────────────────┐
│ 1. 边缘化先验残差                                      │
│    MarginalizationFactor                              │
│    → 保留历史信息,防止信息丢失                           │
│                                                       │
│ 2. IMU 预积分残差 (×10)                                │
│    IMUFactor: 15维残差                                 │
│    [δp(3), δq(3), δv(3), δba(3), δbg(3)]             │
│    → 连接相邻帧 i 和 i+1                               │
│                                                       │
│ 3. 视觉重投影残差 (×N_features)                        │
│    ProjectionFactor/ProjectionTdFactor: 2维残差        │
│    → 连接特征首次观测帧、当前观测帧、外参、逆深度、td      │
│                                                       │
│ 4. 重定位残差 (可选)                                    │
│    → 回环检测匹配点的重投影约束                          │
└─────────────────────────────────────────────────────┘

求解器设置:
- 线性求解器: DENSE_SCHUR (利用 Schur 补消去特征变量)
- 信赖域策略: DOGLEG
- 最大迭代: 8 ~ NUM_ITERATIONS
- 最大求解时间: 0.04s (MARGIN_OLD 帧用 4/5 时间)
- 损失函数: CauchyLoss(1.0) (鲁棒核函数)
```

### 4.7 IMU 预积分 (IntegrationBase)

```
核心思想：预积分消除对初始状态的依赖

输入: 两帧之间的所有 IMU 测量 {(a_k, ω_k, dt_k)}

积分方法: 中点法 (midPointIntegration)
  un_acc_0 = δq × (a_0 - ba) 
  un_gyr   = 0.5 × (ω_0 + ω_1) - bg
  δq_new   = δq ⊗ [1, 0.5×un_gyr×dt]
  un_acc_1 = δq_new × (a_1 - ba)
  un_acc   = 0.5 × (un_acc_0 + un_acc_1)
  δp_new   = δp + δv×dt + 0.5×un_acc×dt²
  δv_new   = δv + un_acc×dt

同时维护:
  - 15×15 协方差矩阵 (用于信息矩阵/权重)
  - 15×15 雅可比矩阵 (用于偏差一阶校正)

偏差更新: 当偏差改变时，用一阶近似快速校正，避免重新积分:
  δp_corrected = δp + J_ba × δba + J_bg × δbg
  δv_corrected = δv + J_ba × δba + J_bg × δbg  
  δq_corrected = δq ⊗ [1, 0.5 × J_bg × δbg]
```

### 4.8 边缘化 (Marginalization)

```
边缘化策略:

MARGIN_OLD (关键帧):
  ┌─────────────────────────────┐
  │ [0] [1] [2] ... [9] [10]   │  滑窗
  │  ↑                          │
  │ 边缘化最老帧                  │
  │ 保留先验信息                  │
  │ → 帧左移: [1]→[0], [2]→[1]  │
  └─────────────────────────────┘

MARGIN_SECOND_NEW (非关键帧):
  ┌─────────────────────────────┐
  │ [0] [1] ... [8] [9] [10]   │
  │                  ↑          │
  │            边缘化次新帧       │
  │ → [10]→[9], 合并预积分      │
  └─────────────────────────────┘

Schur 补实现:
  将要边缘化的变量消去，保留为线性化的先验因子
  J_prior, r_prior → 下一次优化中作为额外残差
```

---

## 五、模块三：pose_graph (回环检测与位姿图优化)

### 5.1 核心文件
| 文件                  | 作用                                   |
| --------------------- | -------------------------------------- |
| `pose_graph_node.cpp` | ROS 节点入口，550+ 行                  |
| `pose_graph.h/cpp`    | **PoseGraph 类**，回环检测 + 4DOF 优化 |
| `keyframe.h/cpp`      | **KeyFrame 类**，关键帧数据结构        |
| `ThirdParty/DBoW/`    | DBoW2 词袋库                           |
| `ThirdParty/DVision/` | BRIEF 描述子提取                       |

### 5.2 回环检测流程

```
vio_callback() → 接收 VIO 位姿
    │
    ▼
process() 线程
    │
    ├── 构建 KeyFrame (包含 3D 点、BRIEF 描述子)
    │
    ├── detectLoop(keyframe)
    │   ├── DBoW2 数据库查询 (找到视觉相似帧)
    │   ├── 时间间距检查 (不能太近)
    │   └── 返回候选回环帧 index
    │
    ├── keyframe->findConnection(old_kf)
    │   ├── BRIEF 描述子匹配 (searchByBRIEFDes)
    │   ├── 基础矩阵 RANSAC (FundmantalMatrixRANSAC)
    │   ├── PnP RANSAC (PnPRANSAC) → 求解相对位姿
    │   └── 需要 ≥ MIN_LOOP_NUM(25) 个匹配点
    │
    ├── 若回环成功:
    │   ├── 记录回环约束 (loop_info: 8×1)
    │   │   [相对位置(3) + 相对四元数(4) + 相对yaw(1)]
    │   └── 触发 optimize4DoF()
    │
    └── addKeyFrameIntoVoc() → 加入词袋数据库
```

### 5.3 4-DOF 位姿图优化

```
optimize4DoF():
  优化变量: 每个关键帧的 yaw(1) + position(3) = 4-DOF
  (pitch 和 roll 由 IMU/重力直接观测，不需要优化)

  约束:
  1. 序列约束 (FourDOFError): 相邻帧之间的相对位姿
  2. 回环约束 (FourDOFWeightError): 回环帧对之间的相对位姿
  
  效果: 消除长距离漂移，尤其是 yaw 漂移

  优化后:
  - 计算 drift (r_drift, t_drift, yaw_drift)
  - 将 drift 应用到所有后续帧
  - 更新所有关键帧位姿
```

---

## 六、模块四：camera_model (相机模型库)

基于 [camodocal](https://github.com/hengli/camodocal)，支持多种相机模型：
- **Pinhole (针孔)**: 标准透视投影 + 径向/切向畸变
- **CataCamera (MEI)**: 全向/鱼眼相机 (Unified Projection Model)
- **Equidistant**: 等距投影鱼眼模型

核心接口:
- `liftProjective(pixel, ray)`: 像素 → 归一化3D射线 (去畸变)
- `spaceToPlane(point3d, pixel)`: 3D点 → 像素 (加畸变)

工具: `CameraCalibration` — 相机标定程序

---

## 七、ROS Topic 通信图

```
                    /cam0/image_raw
                         │
                         ▼
              ┌──────────────────┐
              │  feature_tracker  │
              └──────┬───────────┘
                     │
    /feature_tracker/feature    /feature_tracker/restart
    /feature_tracker/feature_img
                     │                    │
                     ▼                    ▼
              ┌──────────────────────────────┐
   /imu0 ────▶│       vins_estimator          │
              └──────┬───────────────────────┘
                     │
    /vins_estimator/odometry   /vins_estimator/camera_pose
    /vins_estimator/keyframe_point  /vins_estimator/keyframe_pose
    /vins_estimator/imu_propagate   /vins_estimator/extrinsic
    /vins_estimator/relo_relative_pose
                     │
                     ▼
              ┌──────────────────┐
              │    pose_graph     │◀── /pose_graph/match_points
              └──────────────────┘        (反馈给 vins_estimator)
                     │
    /pose_graph/pose_graph_path
    /pose_graph/base_path
    /pose_graph/match_image
```

---

## 八、配置文件详解 (euroc_config.yaml)

```yaml
# ===== 传感器话题 =====
imu_topic: "/imu0"
image_topic: "/cam0/image_raw"

# ===== 相机内参 =====
model_type: PINHOLE
image_width: 752
image_height: 480
distortion_parameters: {k1, k2, p1, p2}
projection_parameters: {fx, fy, cx, cy}

# ===== IMU-相机外参 =====
estimate_extrinsic: 0   # 0=固定, 1=优化, 2=自动标定
extrinsicRotation: imu_R_cam  (3×3)
extrinsicTranslation: imu_T_cam (3×1)

# ===== 特征追踪 =====
max_cnt: 150        # 最大特征数
min_dist: 30        # 最小距离
freq: 10            # 发布频率
F_threshold: 1.0    # RANSAC 阈值
equalize: 1         # 直方图均衡

# ===== 优化参数 =====
max_solver_time: 0.04   # 最大求解时间 (秒)
max_num_iterations: 8   # 最大迭代次数
keyframe_parallax: 10.0 # 关键帧选择视差阈值

# ===== IMU 噪声参数 =====
acc_n: 0.08       # 加速度计测量噪声 σ
gyr_n: 0.004      # 陀螺仪测量噪声 σ
acc_w: 0.00004    # 加速度计偏差随机游走 σ
gyr_w: 2.0e-6     # 陀螺仪偏差随机游走 σ
g_norm: 9.81007   # 重力加速度

# ===== 回环闭合 =====
loop_closure: 1
load_previous_pose_graph: 0
fast_relocalization: 0

# ===== 时间标定 =====
estimate_td: 0      # 是否估计时间偏移
td: 0.0             # 时间偏移初值

# ===== 卷帘快门 =====
rolling_shutter: 0
rolling_shutter_tr: 0
```

---

## 九、关键算法原理总结

### 9.1 初始化流程
```
1. 收集 WINDOW_SIZE+1 帧 (11帧)
2. relativePose() → 找到视差足够 (>30 pixel@f460) 的帧对
3. GlobalSFM → 三角化+BA → 所有帧的SfM位姿
4. 非关键帧 solvePnP() → 获得所有帧位姿
5. VisualIMUAlignment() → 线性求解:
   a. 陀螺仪偏差 bg
   b. 速度 V (每帧3维)
   c. 重力方向 g (2维，约束 |g|=g_norm)
   d. 尺度 s (1维)
6. 应用尺度、对齐重力方向 → 建立世界坐标系
```

### 9.2 关键帧选择策略
```
addFeatureCheckParallax():
  计算新帧与次新帧之间的 "补偿视差" (compensatedParallax2)
  - 如果视差 > MIN_PARALLAX (keyframe_parallax/f)
    → 是关键帧 → MARGIN_OLD
  - 否则
    → 非关键帧 → MARGIN_SECOND_NEW
    
  (直觉: 如果新帧和次新帧太相似，丢掉次新帧保留新帧;
          如果足够不同，丢掉最老帧引入新帧)
```

### 9.3 失败检测条件
```
failureDetection() 返回 true (重启系统) 当:
1. 加速度计偏差 > 2.5 m/s²
2. 陀螺仪偏差 > 1.0 rad/s
3. 单帧位移 > 5m
4. z 方向位移 > 1m
```

---

## 十、代码修改记录

根据之前的对话记录，此项目已有以下修改：

1. **Ceres 兼容性补丁** (对话 `2ef1e017`):  
   - 将 `ceres::LocalParameterization` 替换为 `ceres::Manifold` (Ubuntu 22.04 / Ceres 2.x)
   - 添加 `#if CERES_VERSION_MAJOR >= 2` 条件编译

2. **诊断日志系统** (对话 `89bdb45e`, `1bc66373`):  
   - 在 `feature_tracker_node.cpp` 添加 `[FT]` 标签的跟踪统计日志
   - 在 `estimator_node.cpp` 添加 `[VIO]` 标签的 IMU-图像同步日志
   - 在 `estimator.cpp` 添加 `[EST]` 和 `[OPT]` 标签的估计/优化日志

3. **飞思 150 配置** (launch `feisi_150.launch`, config `feisi_150/feisi_150_imu.yaml`):
   - 针对特定硬件的 IMU 配置

---

## 十一、运行方式

```bash
# 终端 1: 启动系统
roslaunch vins_estimator euroc.launch

# 终端 2: 可视化
roslaunch vins_estimator vins_rviz.launch

# 终端 3: 播放数据集
rosbag play MH_01_easy.bag
```

---

## 十二、文件规模统计

| 模块            | 代码文件数             | 关键文件行数                                     |
| --------------- | ---------------------- | ------------------------------------------------ |
| feature_tracker | 6 文件                 | ~650 行                                          |
| vins_estimator  | 32 文件                | estimator.cpp: 1129行, integration_base.h: 446行 |
| pose_graph      | 36 文件 (含ThirdParty) | pose_graph.cpp: 869行, keyframe.cpp: 550行       |
| camera_model    | 30 文件                | 相机模型库                                       |
| 总计            | ~100 文件              | 核心代码 ~5000 行                                |
