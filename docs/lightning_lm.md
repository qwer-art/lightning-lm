# Lightning-LM 算法详细分析

## 符号说明

本文档使用以下数学符号：
- `a_meas`: 加速度计测量值
- `ω_meas`: 陀螺仪测量值
- `b_g`: 陀螺仪偏差
- `b_a`: 加速度计偏差（lightning-lm中不作为状态量估计）
- `n_g`: 陀螺仪测量噪声
- `n_a`: 加速度计测量噪声
- `g`: 重力向量（lightning-lm中初始化时估计方向，之后固定，不作为状态量在线估计）

## 命名规范

本文档遵循以下命名规范：

1. **旋转矩阵**: `R_XtoY` — 从坐标系 X 到坐标系 Y 的旋转
2. **四元数**: `q_XtoY` — 从坐标系 X 到坐标系 Y 的四元数
3. **位置向量**: `p_XinY` — 坐标系 X 原点在坐标系 Y 中的坐标
4. **外参表示**: `T_LtoI` 表示从LiDAR到IMU的变换

### 代码变量与数学符号对应关系

| 代码变量 | 数学符号 | 含义 |
|---------|---------|------|
| `offset_R_lidar_fixed_` | `R_LtoI` | LiDAR到IMU的旋转（固定外参，不在线估计） |
| `offset_t_lidar_fixed_` | `p_LinI` | LiDAR在IMU中的位置（固定外参，不在线估计） |
| `extrinR_` / `extrinsic_R` | `R_LtoI` | 配置参数中的外参旋转 |
| `extrinT_` / `extrinsic_T` | `p_LinI` | 配置参数中的外参平移 |
| `state_point_.pos_` / `x_.pos_` | `p_IinW` | IMU在世界坐标系中的位置 |
| `state_point_.rot_` / `x_.rot_` | `R_ItoW` | IMU到世界坐标系的旋转 |
| `state_point_.vel_` / `x_.vel_` | `v_IinW` | IMU在世界坐标系中的速度 |
| `state_point_.bg_` / `x_.bg_` | `b_g` | 陀螺仪偏差 |
| `state_point_.grav_` / `x_.grav_` | `g_W` | 重力向量（初始化时估计方向，模长9.81，之后固定不变） |
| `R_lidar_imu_` | `R_LtoI` | ImuProcess中的外参旋转 |
| `t_lidar_mu_` | `p_LinI` | ImuProcess中的外参平移 |

## 坐标系定义

- **I (IMU)**: IMU本体坐标系
- **L (LiDAR)**: 激光雷达坐标系
- **W (世界坐标系)**: EKF状态估计的参考系，定义为IMU在初始化完成后的坐标系。重力方向由初始化时加速度计读数确定，模长固定为9.81m/s²。仅当IMU初始姿态水平时 `g_W = [0, 0, -9.81]ᵀ`

```
IMU测量 → ESKF状态估计(W系) → 直接输出（无重力对齐步骤）

与Fast-LIO2的区别:
- Fast-LIO2: IMU测量 → EKF状态估计(I₀系) → 重力对齐 → 最终输出(W系)
- Lightning-LM: IMU测量 → ESKF状态估计(W系) → 直接输出
- Lightning-LM不在线估计重力方向，初始化时由加速度计读数确定方向后固定
- Lightning-LM不进行I₀→W的重力对齐变换
```

---

## 1. 与Fast-LIO2的核心差异

Lightning-LM 是基于 Fast-LIO2 思路的重写版本，在架构和状态定义上有显著简化：

| 特性 | Fast-LIO2 | Lightning-LM |
|------|-----------|--------------|
| 状态维度 | 23维(流形)/24维(切空间) | **12维** |
| 状态向量 | `[p, R, R_L, p_L, v, b_g, b_a, g]` | `[p, R, v, b_g]` |
| 重力估计 | g ∈ S² 作为状态量估计 | **初始化时估计方向后固定**，`‖g‖ = 9.81` |
| 加速度计偏差 | b_a 作为状态量估计 | **不估计**，`Getba()` 返回零 |
| 外参在线估计 | 可选（extrinsic_est_en） | **不支持**，外参固定 |
| ESKF框架 | 基于MTK宏的ikfom | **手写ESKF**，无宏 |
| 迭代更新 | IEKF | IEKF + **Anderson加速** |
| 观测模型 | 点面ICP | 点面ICP + **点到点ICP** |
| IMU滤波 | 无 | **多级IMU滤波**（中值+移动平均+毛刺检测） |
| 回环检测 | 无 | **基于NDT的回环检测** + 位姿图优化 |
| 定位模式 | 无 | **支持**（NDT定位 + PGO） |
| 2D栅格地图 | 无 | **G2P5栅格地图** |
| 地图存储 | 单一PCD | **分块瓦片地图(TiledMap)** |
| S²流形处理 | 完整Bx/Nx/Mx映射 | **保留S2类但不参与ESKF** |
| 协方差膨胀 | 无 | **预测步膨胀** (1.01×) + **退化膨胀** |
| 更新步保护 | 无 | **最大更新步长限制**（平移0.5m，旋转5°） |
| 退化处理 | 无 | **观测退化检测** + 可观测投影 |

---

## 2. ESKF 状态向量

### 2.1 状态定义 (`nav_state.h:21-168`)

```
x = [p_IinW, R_ItoW, v_IinW, b_g]^T
     ────────  ────────  ────────  ────
       3维       3维       3维     3维
     ────────────────────────────────────
          切空间: 12维
```

**维度说明**：

| 状态分量 | 索引 | 维度 | 说明 |
|---------|------|------|------|
| 位置 p | 0 | 3 | ℝ³ 欧氏空间 |
| 旋转 R | 3 | 3 | SO(3)流形，切空间用旋转向量 |
| 速度 v | 6 | 3 | ℝ³ 欧氏空间 |
| 陀螺仪偏差 b_g | 9 | 3 | ℝ³ 欧氏空间 |
| **总计** | — | **12** | — |

**与Fast-LIO2的关键区别**：
- 无外参旋转 `R_LtoI`（3维）—— 固定为配置值
- 无外参平移 `p_LinI`（3维）—— 固定为配置值
- 无加速度计偏差 `b_a`（3维）—— 不估计
- 无重力 `g`（2维S²）—— 固定为 `[0,0,-9.81]ᵀ`

### 2.2 状态元信息 (`nav_state.cc:9-18`)

```cpp
// ℝ³ 矢量状态
vect_states_ = {
    {kPosIdx=0, dim=0, dof=3},  // pos
    {kVelIdx=6, dim=6, dof=3},  // vel
    {kBgIdx=9,  dim=9, dof=3},  // bg
};

// SO3 状态
SO3_states_ = {
    {kRotIdx=3, dim=3, dof=3},  // rot
};
```

---

## 3. 初始化

### 3.1 IMU 初始化 (`imu_processing.hpp:124-172`)

**条件**: 需要 `max_init_count_ = 20` 帧IMU数据

**初始化步骤**:

1. **重力方向估计**: `grav_ = -mean_acc_ / mean_acc_.norm() * G_m_s2`
   - 静止时 `a_meas ≈ -g_I`，取反后得到重力方向
   - 重力**模长固定为9.81**，**方向由初始静止姿态决定**，之后不再更新
   - 仅当IMU初始水平时 `grav_ = [0, 0, -9.81]ᵀ`

2. **陀螺仪偏差估计**: `bg_ = mean_gyr_`
   - 静止时 `ω_true = 0`，`ω_meas = b_g + n_g`，取平均抑制噪声

3. **加速度计尺度因子** (`imu_processing.hpp:336-344`):
   ```
   if (mean_acc_norm > 0.5 && mean_acc_norm < 1.5):
       acc_scale_factor_ = G_m_s2  // 单位为g的加速度计
   elif (mean_acc_norm > 7.0 && mean_acc_norm < 12.0):
       acc_scale_factor_ = 1.0     // 单位为m/s²的加速度计
   ```

4. **初始协方差矩阵** (`imu_processing.hpp:163-167`):
   ```
   P₀ = I₁₂
   P₀(bg块) = 0.0001 * I₃
   ```

**EKF初始化标志** (`laser_mapping.cc:225`):
```
flg_EKF_inited_ = (lidar_begin_time - first_lidar_time) >= INIT_TIME = 0.1s
```

---

## 4. 预测步 (Prediction Step)

### 4.1 状态微分方程

**核心方程** (`nav_state.h:54-66`):

```
dx/dt = f(x, u, w) ∈ ℝ¹²

┌──────────────────────────────────────────────────────────────────┐
│ dp_IinW/dt = v_IinW                                            │ (位置)
│ dR_ItoW/dt = R_ItoW · [ω]×                                     │ (旋转)
│ dv_IinW/dt = R_ItoW · a_meas + g_W                              │ (速度)
│ db_g/dt    = n_bg                                               │ (陀螺仪偏差)
└──────────────────────────────────────────────────────────────────┘

其中: ω = ω_meas - b_g, g_W = grav_ (初始化时估计，方向由静止加速度计确定，模长9.81，之后固定)
```

**与Fast-LIO2的区别**:
- `dv/dt = R · a_meas + g`，不包含 `-b_a` 项（不估计加速度计偏差）
- `g` 初始化时由加速度计读数确定方向后固定，不作为状态量在线估计
- 无外参微分方程（外参固定）

### 4.2 状态雅可比矩阵 f_x

```
f_x ∈ ℝ¹²ˣ¹²

              p(0:2)   R(3:5)   v(6:8)   bg(9:11)
           ┌──────────────────────────────────────┐
  dp/dt(0) │   O₃      O₃      I₃       O₃       │
  dR/dt(3) │   O₃      O₃      O₃      -I₃       │
  dv/dt(6) │   O₃   -R·[a]×    O₃       O₃       │
  dbg/dt(9)│   O₃      O₃      O₃       O₃       │
           └──────────────────────────────────────┘

其中 a = a_meas (不减b_a), R = R_ItoW
```

**与Fast-LIO2的对比**:
- 无外参列（R_L, p_L列）
- 无 b_a 列
- 无 g 列（Mx矩阵）
- dv/dt 对 R 的雅可比: `-R·hat(a)`，与Fast-LIO2一致
- dR/dt 对 bg 的雅可比: `-I₃`，与Fast-LIO2一致

### 4.3 噪声雅可比矩阵 f_w

```
f_w ∈ ℝ¹²ˣ¹²

              n_g(0:2)  n_a(3:5)  n_bg(6:8)  n_ba(9:11)
           ┌────────────────────────────────────────────┐
  dp/dt(0) │   O₃       O₃        O₃         O₃       │
  dR/dt(3) │  -I₃       O₃        O₃         O₃       │
  dv/dt(6) │   O₃      -R         O₃         O₃       │
  dbg/dt(9)│   O₃       O₃        I₃         O₃       │
           └────────────────────────────────────────────┘
```

**注意**: 虽然 `n_ba` 列全零（不估计b_a），但噪声矩阵仍保持12维以兼容Q矩阵。

### 4.4 预测步伪代码 (`eskf.cc:38-95`)

```cpp
void ESKF::Predict(dt, Q, gyro, acce) {
    // Step 1: 计算状态微分和雅可比
    f_ = x_.get_f(gyro, acce);           // 12×1
    f_x_ = x_.df_dx(acce);              // 12×12
    f_w_ = x_.df_dw();                   // 12×12

    // Step 2: 状态积分（流形上的积分）
    x_.oplus(f_, dt);
    //   - 位置:   p_new = p + v·dt
    //   - 旋转:   R_new = R · Exp(ω·dt)
    //   - 速度:   不更新！(oplus中速度更新被注释掉)
    //   - bg:     bg_new = bg + dbg·dt

    // Step 3: 构建F_x1和f_x_final
    F_x1_ = I₁₂;

    // 3a: ℝ³行的恒等映射 (pos, vel, bg)
    for (vect_state in [pos, vel, bg]):
        f_x_final(idx, :) = f_x_(dim, :)    // 直接复制
        f_w_final(idx, :) = f_w_(dim, :)    // 直接复制

    // 3b: SO3行的A_matrix修正 (rot)
    for (SO3_state in [rot]):
        seg_SO3 = -f_(dim:dim+3) * dt       // -ω·dt
        F_x1_(idx:idx+3, idx:idx+3) = Exp(seg_SO3)  // 旋转传播矩阵
        A = A_matrix(seg_SO3)               // SO3左雅可比
        f_x_final(idx, :) = A * f_x_(dim, :)  // 左乘A修正
        f_w_final(idx, :) = A * f_w_(dim, :)  // 同样修正

    // Step 4: 离散化
    F_k = F_x1_ + f_x_final * dt

    // Step 5: 协方差传播
    P = F_k * P * F_kᵀ + (dt * f_w_final) * Q * (dt * f_w_final)ᵀ
    P *= predict_cov_inflation_  // 1.01× 膨胀
    SymmetrizeAndFloorCovariance(P, min_cov_diag)  // 对称化+下限
}
```

**与Fast-LIO2的关键区别**:
- 无S²流形修正（无Nx/Mx投影）—— 重力不作为状态量
- 预测步后协方差乘以膨胀因子 `1.01` —— 增加系统鲁棒性
- `SymmetrizeAndFloorCovariance`: 对角元素下限 `1e-9`，上限 `100.0`，NaN/Inf 检测

### 4.5 F_k 的完整形式

```
F_k =         p(0:2)     R(3:5)          v(6:8)    bg(9:11)
    ┌──────────────────────────────────────────────────────────┐
  p │  I₃         O₃              I₃·dt      O₃              │
  R │  O₃      Exp(-ω·dt)         O₃       -A_R·dt          │
  v │  O₃     -R·[a]×·dt         I₃         O₃              │
 bg │  O₃         O₃              O₃         I₃              │
    └──────────────────────────────────────────────────────────┘
维度: 12×12
```

**注意**: 速度行中，`dv/dt` 对 `v` 的雅可比为 `O₃`（因为 `dv/dt = R·a + g`，不显含 `v`），离散化后 `F_k(v,v) = I₃`。

---

## 5. 更新步 (Update Step)

### 5.1 观测模型概述

Lightning-LM 支持多种观测类型 (`eskf.hpp:33-39`):

```cpp
enum class ObsType {
    LIDAR,                  // 激光点面+点到点ICP
    WHEEL_SPEED,            // 轮速观测
    WHEEL_SPEED_AND_LIDAR,  // 轮速+Lidar
    ACC_AS_GRAVITY,         // 重力作为加计观测量
    GPS,                    // GPS/RTK
    BIAS,                   // 偏差观测
};
```

开源版本主要使用 `LIDAR` 观测。

### 5.2 激光观测模型 (`laser_mapping.cc:613-800`)

#### 5.2.1 点面ICP部分

**测量方程**:
```
h(x) = nᵀ · p_W + d

其中:
p_W = R_ItoW · (R_LtoI · p_L + p_LinI) + p_IinW
```

**测量雅可比**:
```
H = ∂h/∂x = nᵀ · ∂p_W/∂x

J_i = [nᵀ,  nᵀ·R_ItoW^T·(R_LtoI·[p_L]×),  0,  0]  ∈ ℝ¹ˣ⁶
     └─p─┘  └────────────────R────────────────┘   └v┘ └bg┘
```

**信息矩阵累加**:
```
HTH = Σᵢ Jᵢᵀ · Jᵢ · w_plane    (6×6)
HTr = Σᵢ Jᵢᵀ · rᵢ · w_plane    (6×1)
```

其中 `w_plane = plane_icp_weight_`（默认300.0）。

**残差阈值**: `‖p_body‖² > 81 · pd²`（基于距离的Huber-like权重）

#### 5.2.2 点到点ICP部分（可选）

当 `enable_icp_part_ = true` 时，额外添加点到点约束：

**测量雅可比**:
```
J_i = [I₃,  -(R_ItoW · R_LtoI) · hat(p_L),  0,  0]  ∈ ℝ³ˣ⁶
     └─p─┘  └────────────R────────────────────┘   └v┘ └bg┘
```

**信息矩阵累加**:
```
HTH += Σᵢ Jᵢᵀ · Jᵢ · w_icp    (w_icp = 100)
HTr += Σᵢ Jᵢᵀ · (-eᵢ) · w_icp
```

其中 `eᵢ = p_world - nearest_point`，距离阈值 `0.5m`。

#### 5.2.3 观测退化处理 (`eskf.cc:199-224`)

```
1. 对 HTH 进行特征值分解
2. 计算退化阈值 = max_eigenvalue × degeneracy_threshold_ratio_
3. 构建可观测投影矩阵: 不可观测方向置零
4. HTH_eff = observable_projector · HTH · observable_projector
5. HTr_eff = observable_projector · HTr
6. 退化时膨胀位姿协方差: P(0:5, 0:5) *= degeneracy_cov_inflation_
```

### 5.3 迭代卡尔曼更新伪代码 (`eskf.cc:105-356`)

```cpp
void ESKF::Update(ObsType, R) {
    P_propagated = P;
    dx_current = 0;
    start_x = x_;

    for (i = -1; i < max_iterations_; i++) {
        // Step 1: 计算观测函数
        obs_func_(x_, custom_obs_model_);  // 计算HTH, HTr

        if (!obs.valid_) { x_ = last_x; P_ = P_propagated; return; }

        // Step 2: Anderson加速检查
        if (use_aa_ && i > -1 && residual >= last_res * 1.01) {
            x_ = last_x; break;  // 残差增大，回退
        }

        // Step 3: 计算dx和P的SO3修正
        dx = x_.boxminus(start_x);
        for (SO3_state):
            A_T = A_matrix(dx_SO3).transpose()
            dx_current(SO3) = A_T · dx(SO3)
            P(SO3行) = A_T · P(SO3行)
            P(SO3列) = P(SO3列) · A_T

        // Step 4: 退化检测与可观测投影
        eigen_decompose(HTH)
        observable_mask = eigen_values > threshold
        HTH_eff = projector · HTH · projector
        HTr_eff = projector · HTr

        // Step 5: 计算卡尔曼增益（信息形式）
        P_temp = (P / R).inverse()
        P_temp(0:5, 0:5) += HTH_eff
        Q_inv = P_temp.inverse()
        K_r = Q_inv(0:11, 0:5) · HTr_eff
        K_H(0:11, 0:5) = Q_inv(0:11, 0:5) · HTH_eff

        // Step 6: IEKF迭代更新
        dx_current = K_r + (K_H - I) · dx_current

        // Step 7: 更新步保护
        if (‖dx_translation‖ > 0.5m || ‖dx_rotation‖ > 5°) {
            x_ = start_x; P_ = P_propagated; return;  // 拒绝更新
        }

        // Step 8: 流形上的状态更新
        x_ = x_.boxplus(dx_current)

        // Step 9: Anderson加速
        if (use_aa_):
            dx_all = x_.boxminus(start_x)
            new_dx_all = aa_.compute(dx_all)
            x_ = start_x.boxplus(new_dx_all)

        // Step 10: 收敛判断
        if (converged || last_iteration):
            // 协方差更新 (Joseph形式)
            L_ = P;  // 保存
            for (SO3_state):
                A_T = A_matrix(dx_SO3).transpose()
                L_(SO3行) = A_T · P(SO3行)
                K_H(SO3行) = A_T · K_H(SO3行)
                L_(SO3列) = L_(SO3列) · A_T
                P(SO3列) = P(SO3列) · A_T

            P = L_ - K_H(0:11, 0:5) · P(0:5, 0:11)

            // 退化膨胀
            if (nullity > 0):
                P(0:5, 0:5) *= degeneracy_cov_inflation_

            break;
    }

    SymmetrizeAndFloorCovariance(P, min_cov_diag);
}
```

**与Fast-LIO2的关键区别**:

| 特性 | Fast-LIO2 | Lightning-LM |
|------|-----------|--------------|
| 更新公式 | `dx = K·h + (KH-I)·dx` | `dx = K_r + (K_H-I)·dx_current` |
| 信息形式 | 否 | **是**（累加HTH和HTr） |
| 退化处理 | 无 | **特征值分解 + 可观测投影** |
| 更新步保护 | 无 | **最大平移0.5m，最大旋转5°** |
| Anderson加速 | 无 | **可选**（收敛加速） |
| 协方差膨胀 | 无 | **预测步1.01× + 退化膨胀1.02×** |
| 观测维度 | N×23（逐点） | **6×6信息矩阵累加**（位姿6自由度） |

---

## 6. IMU处理与去畸变

### 6.1 IMU滤波 (`imu_filter.h`)

Lightning-LM 新增多级IMU滤波器，在去畸变前对原始IMU数据进行预处理：

```
原始IMU → 毛刺检测 → 中值滤波 → 移动平均 → 速率限制 → 滤波后IMU
```

| 滤波级 | 方法 | 参数 |
|--------|------|------|
| 1. 毛刺检测 | 与中位数偏差 > `spike_threshold × std` | threshold=3.0σ |
| 2. 中值滤波 | 窗口大小5 | window=5 |
| 3. 移动平均 | 窗口大小3 | window=3 |
| 4. 速率限制 | 角速度变化率 < `rate_limit × dt` | limit=3.0 rad/s² |

### 6.2 点云去畸变 (`imu_processing.hpp:174-312`)

**前向传播**: 对每两个相邻IMU测量之间进行ESKF预测

```
for each (imu_head, imu_tail) in IMU queue:
    angvel_avr = 0.5 * (head.ω + tail.ω)
    acc_avr = 0.5 * (head.a + tail.a) × acc_scale_factor

    kf_state.Predict(dt, Q, gyro, acc)

    // 保存每个IMU时刻的位姿
    imu_pose_.emplace_back(t, acc_s, angvel, vel, pos, R)
```

**后向传播**: 从帧尾到帧头，逐点补偿运动畸变

```
for each point (从后往前):
    dt = point.time - head.offset_time
    R_i = R_imu · Exp(angvel_avr · dt)
    T_ei = pos_imu + vel_imu · dt + 0.5 · acc_imu · dt² - imu_state.pos

    p_compensate = R_LtoIᵀ · (R_ItoW⁻¹ · (R_i · (R_LtoI · P_i + p_LinI) + T_ei) - p_LinI)
```

---

## 7. 局部地图：iVox3D

### 7.1 数据结构 (`ivox3d.h`)

```
IVox = 增量式体素地图
├── 哈希表: grids_map_ (KeyType → list_iterator)
├── LRU缓存: grids_cache_ (双向链表)
├── 每个体素节点: IVoxNode (线性存储点) 或 IVoxNodePhc (PHC存储)
└── 近邻搜索: CENTER / NEARBY6 / NEARBY18 / NEARBY26
```

**关键参数**:
- `resolution_`: 体素分辨率（默认0.2m，配置0.5m）
- `capacity_`: 最大体素数（1,000,000）
- `nearby_type_`: 近邻搜索范围（默认NEARBY18）

### 7.2 增量更新策略 (`laser_mapping.cc:545-604`)

```
for each point in scan_down_world:
    center = floor(point / resolution) × resolution + 0.5 × resolution

    if nearest_points exist:
        if nearest[0] 远离 center:
            point_no_need_downsample  // 直接添加
        else:
            if point 比 nearest[0..4] 更靠近 center:
                points_to_add  // 需要替换
    else:
        points_to_add  // 新区域

ivox_.AddPoints(points_to_add)
ivox_.AddPoints(point_no_need_downsample)
```

**LRU淘汰**: 当 `grids_map_.size() >= capacity_` 时，淘汰链表尾部（最久未访问）的体素。

---

## 8. 关键帧与地图管理

### 8.1 关键帧创建条件 (`laser_mapping.cc:295-308`)

```
if last_kf == nullptr:
    MakeKF()  // 第一帧
elif ‖last_kf.pos - cur.pos‖ > kf_dis_th_ (默认1.0m):
    MakeKF()
elif ‖(last_kf.R⁻¹ · cur.R).log()‖ > kf_angle_th_ (默认10°):
    MakeKF()
elif !slam_mode && (cur_time - last_kf_time) > 2.0s:
    MakeKF()  // 定位模式下的超时关键帧
```

### 8.2 关键帧数据

```cpp
Keyframe {
    int id_;
    CloudPtr cloud_;       // 去畸变后的原始点云
    NavState state_;       // LIO状态
    SE3 lio_pose_;         // LIO位姿 (从state_提取)
    SE3 opt_pose_;         // 优化后位姿 (回环修正)
}
```

**opt_pose 传播**:
```
kf.opt_pose = last_kf.opt_pose · (last_kf.lio_pose⁻¹ · kf.lio_pose)
```

### 8.3 分块瓦片地图 (`tiled_map.h`)

建图完成后，全局地图被分割为多个PCD块：

```
TiledMap
├── ConvertFromFullPCD(global_map, start_pose, save_path)
├── 每块存储:
│   ├── {id}.pcd         // 静态点云
│   ├── {id}_dyn.pcd     // 动态层点云
│   └── index.txt        // 块坐标和起始位姿
├── global.pcd          // 压缩的全局地图
├── map.pgm             // 2D栅格地图图像
└── map.yaml            // 栅格地图元数据
```

---

## 9. 回环检测

### 9.1 回环候选检测 (`loop_closing.cc:96-143`)

```
条件1: cur_kf.id - last_loop_kf.id > loop_kf_gap_ (默认20)
条件2: |kf.id - cur_kf.id| >= closest_id_th_ (默认50)
条件3: ‖kf.opt_pose.translation - cur_kf.opt_pose.translation‖₂D < max_range_ (默认20m)
条件4: 同条轨迹内间隔 > min_id_interval_ (默认20)
```

### 9.2 回环位姿计算 (`loop_closing.cc:168-251`)

**多分辨率NDT匹配**:
```
resolutions = [10.0, 5.0, 2.0, 1.0]  // 从粗到精

for each resolution r:
    ndt.setResolution(r)
    ndt.setInputTarget(voxel_grid(submap_kf1, r*0.1))
    ndt.setInputSource(voxel_grid(submap_kf2, r*0.1))
    ndt.align(output, initial_guess)
    initial_guess = ndt.getFinalTransformation()

score = ndt.getTransformationProbability()
```

**子地图构建**: 以候选帧为中心，前后 `submap_idx_range=40` 个关键帧，每4帧取1帧。

### 9.3 位姿图优化 (`loop_closing.cc:253-348`)

**优化框架**: 自研图优化库 `miao`（类g2o）

**顶点**: `VertexSE3` — 每个关键帧的位姿

**边**:
1. **运动边**: 相邻关键帧间的相对位姿约束
   - 信息矩阵: `info_motion_ = diag(1/σ_t², 1/σ_R²)`，σ_t=0.1m, σ_R=3°
2. **回环边**: 回环检测的相对位姿约束
   - 信息矩阵: `info_loops_ = diag(1/σ_t², 1/σ_R²)`，σ_t=0.2m, σ_R=3°
   - 鲁棒核: Cauchy，阈值 `rk_loop_th_`
3. **高度边**（可选）: 高度先验约束
   - 信息矩阵: `1/σ_h²`，σ_h=0.1m

**优化流程**:
```
1. 添加顶点和边
2. LM优化 20次迭代
3. 异常值剔除: χ² > delta 的边设为level=1
4. 读取优化结果: 更新所有关键帧的opt_pose
5. 触发回调: 重绘2D栅格地图
```

---

## 10. 定位模式

### 10.1 定位架构 (`localization.h`)

```
Localization
├── LaserMapping (LIO前端)
│   ├── 提供激光里程计
│   └── is_in_slam_mode_ = false
├── LidarLoc (激光定位)
│   ├── NDT-OMP匹配
│   └── 网格角度搜索
├── PGO (位姿图优化)
│   ├── 滑窗优化
│   └── 平滑器
└── AsyncMessageProcess
    ├── lidar_odom_proc_cloud_  // 异步里程计处理
    └── lidar_loc_proc_cloud_   // 异步定位处理
```

### 10.2 定位流程

```
IMU → LIO前端 → 激光里程计 → PGO(滑窗)
                           ↓
点云 → LidarLoc(NDT) → 全局定位 → PGO(全局约束)
                                       ↓
                                    TF发布
```

---

## 11. G2P5 栅格地图

### 11.1 概述

G2P5 是 Lightning-LM 自研的2D占据栅格地图模块，用于：

1. **建图时**: 实时生成2D导航地图
2. **回环时**: 回环优化后重绘全局地图
3. **保存时**: 输出ROS兼容的 `.pgm` + `.yaml` 格式

### 11.2 参数

```yaml
g2p5:
  esti_floor: false       # 是否动态估计地面
  lidar_height: 0.0       # 雷达安装高度
  floor_height: -1.2      # 默认地面高度
  min_th_floor: 0.50      # 障碍物最小高度
  max_th_floor: 1.0       # 障碍物最大高度
  grid_map_resolution: 0.2 # 栅格地图分辨率
```

---

## 12. 通讯结构

### 12.1 ROS2 Topic 列表

| Topic名称 | 消息类型 | 发布/订阅 | 说明 |
|-----------|---------|----------|------|
| `{imu_topic}` | `sensor_msgs/msg/Imu` | 订阅 | IMU数据（可配置） |
| `{lidar_topic}` | `sensor_msgs/msg/PointCloud2` | 订阅 | 标准点云（Velodyne/Ouster/RoboSense） |
| `{livox_lidar_topic}` | `livox_ros_driver2/msg/CustomMsg` | 订阅 | Livox自定义点云 |
| `lightning/save_map` | `lightning/srv/SaveMap` | 服务 | 保存地图 |
| `lightning/loc_cmd` | `lightning/srv/LocCmd` | 服务 | 定位命令 |
| TF (map→odom) | `geometry_msgs/TransformStamped` | 发布 | 定位结果（仅定位模式） |

### 12.2 通讯结构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Lightning-LM SLAM系统                         │
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌───────────┐   ┌──────────┐  │
│  │ IMU Sub  │───→│ ImuProcess  │───→│   ESKF    │   │ LoopClsg │  │
│  └──────────┘    │  ·滤波      │    │  ·Predict │   │  ·NDT    │  │
│                  │  ·去畸变     │    │  ·Update  │←──│  ·PGO    │  │
│  ┌──────────┐    └──────────────┘    │  ·NavState│   └──────────┘  │
│  │Cloud Sub │───→│ Preprocess  │───→│           │        ↑          │
│  │(PC2/Liv)│    │  ·格式统一   │    └─────┬─────┘        │          │
│  └──────────┘    │  ·ROI过滤   │          │              │          │
│                  │  ·降采样     │          ↓              │          │
│                  └──────────────┘    ┌───────────┐        │          │
│                                      │ LaserMapping│───────┘        │
│                                      │  ·ObsModel │──→ Keyframe    │
│                                      │  ·iVox3D   │               │
│                                      │  ·MapIncr  │               │
│                                      └─────┬─────┘               │
│                                            │                       │
│                  ┌──────────┐              │                       │
│                  │  G2P5    │←─────────────┘                       │
│                  │  ·2D栅格 │                                      │
│                  └──────────┘                                      │
│                                                                     │
│  ┌──────────┐                                                       │
│  │SaveMap   │←── ros2 service call                                  │
│  │Service   │──→ TiledMap ──→ PCD + PGM + YAML                     │
│  └──────────┘                                                       │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                       Lightning-LM 定位系统                          │
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌───────────┐                 │
│  │ IMU Sub  │───→│ LaserMapping │───→│   PGO     │──→ TF发布       │
│  └──────────┘    │  (LIO前端)   │    │  ·滑窗    │                 │
│                  └──────────────┘    │  ·平滑    │                 │
│  ┌──────────┐    ┌──────────────┐    └─────↑─────┘                 │
│  │Cloud Sub │───→│  LidarLoc    │────────┘                         │
│  └──────────┘    │  ·NDT-OMP    │                                  │
│                  │  ·网格搜索   │                                  │
│                  └──────────────┘                                  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 13. 外参变量映射表

| 代码变量 | 数学符号 | 配置参数 | 是否在线估计 |
|---------|---------|---------|------------|
| `offset_R_lidar_fixed_` | `R_LtoI` | `extrinsic_R` (9个元素，行优先) | 否（固定） |
| `offset_t_lidar_fixed_` | `p_LinI` | `extrinsic_T` (3个元素) | 否（固定） |
| `R_lidar_imu_` | `R_LtoI` | 同上 | 否 |
| `t_lidar_mu_` | `p_LinI` | 同上 | 否 |

**注意**: Lightning-LM 不支持外参在线估计（`extrinsic_est_en` 参数在配置中存在但代码中未实现）。

---

## 14. 配置参数汇总

### 14.1 ESKF参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `max_iteration` | 4 | IEKF最大迭代次数 |
| `esti_plane_threshold` | 0.1 | 平面估计阈值 |
| `acc_cov` | 0.1 | 加速度计测量噪声 |
| `gyr_cov` | 0.1 | 陀螺仪测量噪声 |
| `b_acc_cov` | 0.0001 | 加速度计偏差噪声（未使用） |
| `b_gyr_cov` | 0.0001 | 陀螺仪偏差噪声 |
| `use_aa` | false | 是否使用Anderson加速 |
| `predict_cov_inflation_` | 1.01 | 预测步协方差膨胀 |
| `degeneracy_threshold_ratio_` | 1e-3 | 退化检测阈值比 |
| `degeneracy_cov_inflation_` | 1.02 | 退化协方差膨胀 |
| `max_update_translation_step_` | 0.5m | 最大更新平移步长 |
| `max_update_rotation_step_deg_` | 5° | 最大更新旋转步长 |

### 14.2 地图参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `ivox_grid_resolution` | 0.5 | iVox体素分辨率 |
| `ivox_nearby_type` | 18 | 近邻搜索范围 |
| `filter_size_scan` | 0.5 | 扫描降采样分辨率 |
| `filter_size_map` | 0.5 | 地图降采样分辨率 |
| `kf_dis_th` | 1.0m | 关键帧距离阈值 |
| `kf_angle_th` | 10° | 关键帧角度阈值 |

### 14.3 回环参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `loop_kf_gap` | 20 | 回环检查间隔 |
| `min_id_interval` | 20 | 最小ID间隔 |
| `closest_id_th` | 50 | 最近ID阈值 |
| `max_range` | 20.0m | 候选帧最大距离 |
| `ndt_score_th` | 1.3 | NDT匹配分数阈值 |
| `motion_trans_noise` | 0.1m | 运动边位移噪声 |
| `motion_rot_noise` | 3° | 运动边旋转噪声 |
| `loop_trans_noise` | 0.2m | 回环边位移噪声 |
| `loop_rot_noise` | 3° | 回环边旋转噪声 |

---

## 15. 数学公式汇总

### 15.1 ESKF预测步

```
状态传播:
  p_{k+1} = p_k + v_k · dt
  R_{k+1} = R_k · Exp((ω_meas - b_g) · dt)
  v_{k+1} = v_k + (R_k · a_meas + g) · dt    (注意: 不减b_a, g为初始化时估计的重力向量)
  b_g_{k+1} = b_g_k                            (随机游走，由噪声驱动)

协方差传播:
  F_k = F_x1 + f_x_final · dt
  P_{k+1} = F_k · P_k · F_kᵀ + (dt · f_w_final) · Q · (dt · f_w_final)ᵀ
  P_{k+1} *= inflation_factor  (1.01)
```

### 15.2 ESKF更新步

```
信息形式IEKF:
  HTH = Σᵢ Jᵢᵀ · Jᵢ · w    (6×6, 仅位姿6自由度)
  HTr = Σᵢ Jᵢᵀ · rᵢ · w    (6×1)

  P_temp = (P / R)⁻¹
  P_temp(0:5, 0:5) += HTH_eff
  Q_inv = P_temp⁻¹

  K_r = Q_inv(0:11, 0:5) · HTr_eff
  K_H = Q_inv(0:11, 0:5) · HTH_eff

  dx = K_r + (K_H - I) · dx_current
  x = x ⊞ dx

  P = L - K_H · P(0:5, 0:11)
```

### 15.3 点云去畸变

```
前向传播:
  ω_avr = 0.5 · (ω_head + ω_tail)
  a_avr = 0.5 · (a_head + a_tail) × scale
  kf.Predict(dt, Q, ω_avr, a_avr)

后向补偿:
  R_i = R_imu · Exp(ω_avr · dt)
  T_ei = pos_imu + vel_imu · dt + 0.5 · acc_imu · dt² - pos_end
  p_comp = R_LtoIᵀ · (R_ItoW⁻¹ · (R_i · (R_LtoI · p_L + p_LinI) + T_ei) - p_LinI)
```

---

## 参考文献

1. Xu, W., Zhang, F., et al. (2021). "Fast LiDAR-Inertial Odometry." *IEEE Robotics and Automation Letters*.
2. Sola, J. (2017). "Quaternion kinematics for the error-state Kalman filter." *arXiv preprint*.
3. Anderson, D.G. (1965). "Iterative Procedures for Nonlinear Integral Equations." *Journal of the ACM*.
4. Bloesch, M., et al. (2016). "Iterated Extended Kalman Filter on Lie Groups." *Journal of Field Robotics*.