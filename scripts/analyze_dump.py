#!/usr/bin/env python3
"""
Lightning-LM dump 数据分析脚本

用法:
    python3 analyze_dump.py <dump_dir> <output_dir>

    dump_dir:   原始 dump 数据目录 (只读)
    output_dir: 分析结果输出目录
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# matplotlib 中文支持
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


# ── 数据加载 ──────────────────────────────────────────────────────────────

STATE_COLS = [
    'timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw',
    'vx', 'vy', 'vz', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz',
    'gx', 'gy', 'gz', 'confidence'
]


def load_state_file(filepath):
    data = np.loadtxt(filepath, comments='#')
    if data.ndim == 1:
        data = data.reshape(1, -1)
    dtype = [(name, float) for name in STATE_COLS]
    arr = np.zeros(data.shape[0], dtype=dtype)
    for i, name in enumerate(STATE_COLS):
        arr[name] = data[:, i]
    return arr


def quat_to_euler(qx, qy, qz, qw):
    rot = Rotation.from_quat(np.stack([qx, qy, qz, qw], axis=-1))
    euler = rot.as_euler('xyz', degrees=True)
    return euler[:, 0], euler[:, 1], euler[:, 2]


def compute_trajectory_length(tx, ty, tz):
    diffs = np.diff(np.stack([tx, ty, tz], axis=-1), axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))


def moving_avg(data, win):
    """滑动均值，边界用 mirror padding"""
    if win < 2 or len(data) < win:
        return data
    kernel = np.ones(win) / win
    # mirror padding
    pad = win // 2
    padded = np.concatenate([data[pad:0:-1], data, data[-2:-pad-2:-1]])
    return np.convolve(padded, kernel, mode='valid')[:len(data)]


# ── predict/update 对齐与偏差计算 ──────────────────────────────────────────

def compute_predict_update_diff(predict, update):
    """
    对齐 predict 和 update 时间戳，计算偏差。
    对每个 update 时间戳 t，找 predict 中 <= t 的最近预测值，计算差值。
    角度偏差使用 SO3 rotation magnitude，避免欧拉角回绕。
    """
    pred_ts = predict['timestamp']
    upd_ts = update['timestamp']

    pred_idx = np.searchsorted(pred_ts, upd_ts, side='right') - 1
    valid = pred_idx >= 0
    pred_idx = pred_idx[valid]
    upd_idx = np.where(valid)[0]

    if len(upd_idx) == 0:
        return None

    # 位置偏差
    p_upd = np.stack([update['tx'][upd_idx], update['ty'][upd_idx], update['tz'][upd_idx]], axis=-1)
    p_pred = np.stack([predict['tx'][pred_idx], predict['ty'][pred_idx], predict['tz'][pred_idx]], axis=-1)
    pos_diff = p_upd - p_pred

    # 角度偏差: SO3 rotation magnitude (避免欧拉角回绕)
    R_upd = Rotation.from_quat(np.stack([
        update['qx'][upd_idx], update['qy'][upd_idx],
        update['qz'][upd_idx], update['qw'][upd_idx]], axis=-1))
    R_pred = Rotation.from_quat(np.stack([
        predict['qx'][pred_idx], predict['qy'][pred_idx],
        predict['qz'][pred_idx], predict['qw'][pred_idx]], axis=-1))
    angle_diff = (R_upd * R_pred.inv()).magnitude() * 180.0 / np.pi  # degrees

    ts = update['timestamp'][upd_idx]
    return ts, pos_diff, angle_diff


# ── 健康指标计算 ──────────────────────────────────────────────────────────

def compute_health_metrics(update, diff_result):
    metrics = {}
    n = len(update)
    head = max(1, n // 10)
    tail = max(1, n // 10)

    # 1. bg 稳定性 (bg 是随机游走量，不会收敛，应关注是否有界、是否平滑)
    bg_metrics = {}
    for name, data in [('bgx', update['bgx']), ('bgy', update['bgy']), ('bgz', update['bgz'])]:
        dbg = np.diff(data)
        bg_metrics[name] = {
            'global_mean': float(np.mean(data)),
            'global_std': float(np.std(data)),
            'range': float(np.max(data) - np.min(data)),
            'diff_mean': float(np.mean(np.abs(dbg))),
            'diff_std': float(np.std(dbg)),
            'diff_p95': float(np.percentile(np.abs(dbg), 95)),
            'bounded': float(np.max(data) - np.min(data)) < 0.05,  # 变化范围 < 0.05 rad/s
            'smooth': float(np.percentile(np.abs(dbg), 95)) < 0.001,  # 帧间变化 P95 < 0.001 rad/s
        }
    metrics['bg'] = bg_metrics

    # 2. gravity 估计
    gx, gy, gz = update['gx'], update['gy'], update['gz']
    g_norm = np.sqrt(gx**2 + gy**2 + gz**2)
    metrics['gravity'] = {
        'norm_mean': float(np.mean(g_norm)),
        'norm_std': float(np.std(g_norm)),
        'norm_p95': float(np.percentile(np.abs(g_norm - 9.81), 95)),
        'tail_norm_mean': float(np.mean(g_norm[-tail:])),
        'tail_norm_std': float(np.std(g_norm[-tail:])),
        'stable': float(np.std(g_norm[-tail:])) < 0.1
    }

    # 3. predict/update 偏差
    if diff_result is not None:
        _, pos_diff, angle_diff = diff_result
        pos_norm = np.linalg.norm(pos_diff, axis=1)
        metrics['pred_update_diff'] = {
            'pos_mean': float(np.mean(pos_norm)),
            'pos_std': float(np.std(pos_norm)),
            'pos_max': float(np.max(pos_norm)),
            'pos_p95': float(np.percentile(pos_norm, 95)),
            'angle_mean': float(np.mean(angle_diff)),
            'angle_std': float(np.std(angle_diff)),
            'angle_max': float(np.max(angle_diff)),
            'angle_p95': float(np.percentile(angle_diff, 95)),
        }
    else:
        metrics['pred_update_diff'] = None

    # 4. velocity 平滑性
    vx, vy, vz = update['vx'], update['vy'], update['vz']
    v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
    dv = np.diff(v_norm)
    metrics['velocity'] = {
        'mean_speed': float(np.mean(v_norm)),
        'max_speed': float(np.max(v_norm)),
        'jerk_mean': float(np.mean(np.abs(dv))),
        'jerk_p95': float(np.percentile(np.abs(dv), 95)),
        'smooth': float(np.percentile(np.abs(dv), 95)) < 0.5
    }

    # 5. 轨迹跳变检测
    pos = np.stack([update['tx'], update['ty'], update['tz']], axis=-1)
    dt = np.diff(update['timestamp'])
    dt[dt == 0] = 1e-6
    dp = np.diff(pos, axis=0)
    frame_speed = np.linalg.norm(dp, axis=1) / dt
    pos_jumps = np.where(frame_speed > 2.0)[0]
    angle_jumps = np.where(frame_speed > 5.0)[0]  # > 5 m/s 为严重跳变

    metrics['trajectory'] = {
        'pos_jump_count': int(len(pos_jumps)),
        'severe_jump_count': int(len(angle_jumps)),
        'pos_jump_indices': pos_jumps.tolist()[:20],
        'healthy': len(pos_jumps) == 0
    }

    # 6. confidence
    conf = update['confidence']
    metrics['confidence'] = {
        'mean': float(np.mean(conf)),
        'zero_ratio': float(np.sum(conf == 0) / len(conf)),
        'low_ratio': float(np.sum(conf < 0.3) / len(conf)),
    }

    return metrics


# ── 绘图 ──────────────────────────────────────────────────────────────────

def plot_trajectory_analysis(update, predict, output_dir):
    fig, axes = plt.subplots(3, 2, figsize=(16, 18))
    fig.suptitle('Lightning-LM 轨迹分析', fontsize=16, fontweight='bold')

    tx, ty, tz = update['tx'], update['ty'], update['tz']
    traj_len = compute_trajectory_length(tx, ty, tz)

    # 子图1: XY 轨迹
    ax = axes[0, 0]
    ax.plot(tx, ty, 'b-', linewidth=0.5, alpha=0.8)
    ax.plot(tx[0], ty[0], 'go', markersize=10, label='起点')
    ax.plot(tx[-1], ty[-1], 'rs', markersize=10, label='终点')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'XY 轨迹 (长度: {traj_len:.2f} m)')
    ax.legend()
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)

    # 子图2: XZ 轨迹
    ax = axes[0, 1]
    ax.plot(tx, tz, 'b-', linewidth=0.5, alpha=0.8)
    ax.plot(tx[0], tz[0], 'go', markersize=10, label='起点')
    ax.plot(tx[-1], tz[-1], 'rs', markersize=10, label='终点')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title(f'XZ 轨迹 (长度: {traj_len:.2f} m)')
    ax.legend()
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)

    # 子图3: 欧拉角
    ax = axes[1, 0]
    t_rel = update['timestamp'] - update['timestamp'][0]
    roll, pitch, yaw = quat_to_euler(update['qx'], update['qy'], update['qz'], update['qw'])
    ax.plot(t_rel, roll, 'r-', linewidth=0.3, alpha=0.5)
    ax.plot(t_rel, pitch, 'g-', linewidth=0.3, alpha=0.5)
    ax.plot(t_rel, yaw, 'b-', linewidth=0.3, alpha=0.5)
    # 滑动均值
    win = max(1, len(t_rel) // 50)
    ax.plot(t_rel, moving_avg(roll, win), 'r-', linewidth=1.2, label='Roll')
    ax.plot(t_rel, moving_avg(pitch, win), 'g-', linewidth=1.2, label='Pitch')
    ax.plot(t_rel, moving_avg(yaw, win), 'b-', linewidth=1.2, label='Yaw')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角度 (deg)')
    ax.set_title('Update 欧拉角')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 子图4: bg
    ax = axes[1, 1]
    bgx, bgy, bgz = update['bgx'], update['bgy'], update['bgz']
    ax.plot(t_rel, bgx, 'r-', linewidth=0.3, alpha=0.4)
    ax.plot(t_rel, bgy, 'g-', linewidth=0.3, alpha=0.4)
    ax.plot(t_rel, bgz, 'b-', linewidth=0.3, alpha=0.4)
    # 滑动均值
    ax.plot(t_rel, moving_avg(bgx, win), 'r-', linewidth=1.2,
            label=f'bgx (μ={np.mean(bgx):.4f}, σ={np.std(bgx):.4f})')
    ax.plot(t_rel, moving_avg(bgy, win), 'g-', linewidth=1.2,
            label=f'bgy (μ={np.mean(bgy):.4f}, σ={np.std(bgy):.4f})')
    ax.plot(t_rel, moving_avg(bgz, win), 'b-', linewidth=1.2,
            label=f'bgz (μ={np.mean(bgz):.4f}, σ={np.std(bgz):.4f})')
    ax.axhline(np.mean(bgx), color='r', linestyle='--', alpha=0.5)
    ax.axhline(np.mean(bgy), color='g', linestyle='--', alpha=0.5)
    ax.axhline(np.mean(bgz), color='b', linestyle='--', alpha=0.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('零偏 (rad/s)')
    ax.set_title('陀螺仪零偏 (bg)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 子图5/6: predict/update 偏差
    diff_result = compute_predict_update_diff(predict, update)
    if diff_result is not None:
        diff_ts, pos_diff, angle_diff = diff_result
        diff_t_rel = diff_ts - update['timestamp'][0]
        pos_norm = np.linalg.norm(pos_diff, axis=1)

        # 子图5: 位置偏差
        ax = axes[2, 0]
        ax.plot(diff_t_rel, pos_diff[:, 0], 'r-', linewidth=0.3, alpha=0.4)
        ax.plot(diff_t_rel, pos_diff[:, 1], 'g-', linewidth=0.3, alpha=0.4)
        ax.plot(diff_t_rel, pos_diff[:, 2], 'b-', linewidth=0.3, alpha=0.4)
        ax.plot(diff_t_rel, pos_norm, 'k-', linewidth=0.3, alpha=0.4)
        # 滑动均值
        win_diff = max(1, len(diff_t_rel) // 50)
        ax.plot(diff_t_rel, moving_avg(pos_diff[:, 0], win_diff), 'r-', linewidth=1.2, label='dx')
        ax.plot(diff_t_rel, moving_avg(pos_diff[:, 1], win_diff), 'g-', linewidth=1.2, label='dy')
        ax.plot(diff_t_rel, moving_avg(pos_diff[:, 2], win_diff), 'b-', linewidth=1.2, label='dz')
        ax.plot(diff_t_rel, moving_avg(pos_norm, win_diff), 'k-', linewidth=1.5,
                label=f'||d|| (均值={np.mean(pos_norm):.4f}m)')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('位置偏差 (m)')
        ax.set_title('Predict/Update 位置偏差')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 子图6: 角度偏差 (SO3 rotation magnitude)
        ax = axes[2, 1]
        ax.plot(diff_t_rel, angle_diff, color='purple', linewidth=0.3, alpha=0.4)
        ax.plot(diff_t_rel, moving_avg(angle_diff, win_diff), color='purple', linewidth=1.5,
                label=f'角度偏差 (均值={np.mean(angle_diff):.3f}°)')
        ax.axhline(np.mean(angle_diff), color='purple', linestyle='--', alpha=0.5)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('角度偏差 (deg)')
        ax.set_title('Predict/Update 角度偏差 (SO3)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    else:
        for i in range(2):
            axes[2, i].text(0.5, 0.5, '无 predict/update 重叠数据', ha='center', va='center', transform=axes[2, i].transAxes)

    plt.tight_layout()
    path = os.path.join(output_dir, 'trajectory_analysis.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved: {path}')
    return diff_result


def plot_bg_convergence(update, output_dir):
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('陀螺仪零偏稳定性分析', fontsize=16, fontweight='bold')

    t_rel = update['timestamp'] - update['timestamp'][0]
    bgx, bgy, bgz = update['bgx'], update['bgy'], update['bgz']
    bg_names = ['bgx', 'bgy', 'bgz']
    bg_data = [bgx, bgy, bgz]
    colors = ['r', 'g', 'b']

    for i, (name, data, color) in enumerate(zip(bg_names, bg_data, colors)):
        # 左列: bg 时序 + 滑动均值
        ax = axes[i, 0]
        ax.plot(t_rel, data, color=color, linewidth=0.3, alpha=0.3, label='原始值')

        win = max(1, len(data) // 50)
        smooth = moving_avg(data, win)
        ax.plot(t_rel, smooth, color='k', linewidth=1.5, label=f'滑动均值 (win={win})')

        mean_val = np.mean(data)
        ax.axhline(mean_val, color=color, linestyle='--', alpha=0.7, label=f'全局均值={mean_val:.5f}')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel(f'{name} (rad/s)')
        ax.set_title(f'{name} 时序 (范围={np.max(data)-np.min(data):.5f})')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 右列: 帧间变化 (db/dt)
        ax = axes[i, 1]
        dbg = np.diff(data)
        dt = np.diff(update['timestamp'])
        dt[dt == 0] = 1e-6
        dbg_rate = dbg / dt  # rad/s^2
        ax.plot(t_rel[1:], dbg, color=color, linewidth=0.3, alpha=0.3, label='帧间变化')
        ax.plot(t_rel[1:], moving_avg(dbg, win), color='k', linewidth=1.2, label='滑动均值')
        ax.axhline(0, color='gray', linestyle='-', alpha=0.3)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel(f'Δ{name} (rad/s)')
        ax.set_title(f'{name} 帧间变化 (P95={np.percentile(np.abs(dbg), 95):.6f})')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(output_dir, 'bg_convergence.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved: {path}')


def plot_health_overview(update, output_dir):
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('LIO 健康指标', fontsize=16, fontweight='bold')

    t_rel = update['timestamp'] - update['timestamp'][0]
    win = max(1, len(t_rel) // 50)

    # 子图1: gravity norm
    ax = axes[0, 0]
    gx, gy, gz = update['gx'], update['gy'], update['gz']
    g_norm = np.sqrt(gx**2 + gy**2 + gz**2)
    ax.plot(t_rel, g_norm, 'b-', linewidth=0.3, alpha=0.4)
    ax.plot(t_rel, moving_avg(g_norm, win), 'b-', linewidth=1.2, label=f'滑动均值')
    ax.axhline(9.81, color='r', linestyle='--', alpha=0.7, label='9.81 m/s²')
    ax.axhline(np.mean(g_norm), color='g', linestyle='--', alpha=0.7, label=f'均值={np.mean(g_norm):.3f}')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('重力模长 (m/s²)')
    ax.set_title(f'重力估计 (std={np.std(g_norm):.4f})')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 子图2: velocity
    ax = axes[0, 1]
    vx, vy, vz = update['vx'], update['vy'], update['vz']
    v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
    ax.plot(t_rel, v_norm, 'b-', linewidth=0.3, alpha=0.4)
    ax.plot(t_rel, moving_avg(v_norm, win), 'b-', linewidth=1.2, label='速度')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.set_title(f'速度 (均值={np.mean(v_norm):.2f} m/s, 最大={np.max(v_norm):.2f} m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 子图3: 帧间位置变化
    ax = axes[1, 0]
    pos = np.stack([update['tx'], update['ty'], update['tz']], axis=-1)
    dt = np.diff(update['timestamp'])
    dt[dt == 0] = 1e-6
    dp = np.diff(pos, axis=0)
    frame_speed = np.linalg.norm(dp, axis=1) / dt
    ax.plot(t_rel[1:], frame_speed, 'b-', linewidth=0.3, alpha=0.4)
    ax.plot(t_rel[1:], moving_avg(frame_speed, win), 'b-', linewidth=1.2, label='帧间速度')
    ax.axhline(2.0, color='r', linestyle='--', alpha=0.7, label='跳变阈值 (2 m/s)')
    jump_count = np.sum(frame_speed > 2.0)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('帧间速度 (m/s)')
    ax.set_title(f'帧间速度 (跳变数: {jump_count})')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 子图4: confidence
    ax = axes[1, 1]
    conf = update['confidence']
    ax.plot(t_rel, conf, 'b-', linewidth=0.3, alpha=0.4)
    ax.plot(t_rel, moving_avg(conf, win), 'b-', linewidth=1.2, label='置信度')
    ax.axhline(0.3, color='r', linestyle='--', alpha=0.7, label='低阈值 (0.3)')
    low_ratio = np.sum(conf < 0.3) / len(conf)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('置信度')
    ax.set_title(f'置信度 (低比例: {low_ratio:.2%})')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(output_dir, 'health_overview.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved: {path}')


# ── 分析报告 ──────────────────────────────────────────────────────────────

def generate_report(update, predict, metrics, output_dir):
    tx, ty, tz = update['tx'], update['ty'], update['tz']
    traj_len = compute_trajectory_length(tx, ty, tz)
    duration = update['timestamp'][-1] - update['timestamp'][0]

    lines = []
    lines.append('# Lightning-LM Dump 分析报告\n')
    lines.append('## 概览\n')
    lines.append(f'- 运行时长: {duration:.2f} s')
    lines.append(f'- Update 帧数: {len(update)}')
    lines.append(f'- Predict 帧数: {len(predict)}')
    lines.append(f'- 轨迹长度: {traj_len:.2f} m')
    lines.append('')

    # ── 1. bg 稳定性 ──
    bg = metrics['bg']
    lines.append('## 1. 陀螺仪零偏 (bg) 稳定性\n')
    lines.append('> bg 是随机游走量 (bg_dot = n_bg)，不会收敛到固定值，而是持续游走。')
    lines.append('> 健康指标：有界（不发散）、平滑（无突变）、变化速率与 `b_gyr_cov` 一致。\n')
    lines.append('| 轴 | 全局均值 | 全局标准差 | 变化范围 | 帧间变化均值 | 帧间变化P95 | 有界 | 平滑 |')
    lines.append('|----|----------|------------|----------|-------------|-------------|------|------|')
    for name in ['bgx', 'bgy', 'bgz']:
        s = bg[name]
        lines.append(
            f'| {name} | {s["global_mean"]:.5f} | {s["global_std"]:.5f} | '
            f'{s["range"]:.5f} | {s["diff_mean"]:.6f} | {s["diff_p95"]:.6f} | '
            f'{"是" if s["bounded"] else "否"} | {"是" if s["smooth"] else "否"} |'
        )
    lines.append('')
    lines.append('**判据**: 有界 = 变化范围 < 0.05 rad/s; 平滑 = 帧间变化 P95 < 0.001 rad/s\n')

    all_bounded = all(bg[n]['bounded'] for n in ['bgx', 'bgy', 'bgz'])
    all_smooth = all(bg[n]['smooth'] for n in ['bgx', 'bgy', 'bgz'])
    if all_bounded and all_smooth:
        lines.append('**结论**: 三轴零偏有界且平滑，随机游走行为正常。\n')
    elif all_bounded:
        lines.append('**结论**: 三轴零偏有界但帧间变化较大，可能存在噪声或振荡。\n')
    else:
        not_bounded = [n for n in ['bgx', 'bgy', 'bgz'] if not bg[n]['bounded']]
        lines.append(f'**结论**: {", ".join(not_bounded)} 变化范围过大，可能发散，检查 IMU 数据质量。\n')

    lines.append('**参数调优建议**:\n')
    lines.append('- bg 变化范围过大（发散）: 减小 `b_gyr_cov`（零偏过程噪声），抑制游走幅度')
    lines.append('- bg 帧间变化大（不平滑）: 减小 `b_gyr_cov`，或减小 `gyr_cov`（测量噪声）')
    lines.append('- bg 几乎不变: 可适当增大 `b_gyr_cov`，让滤波器更快跟踪真实零偏变化')
    lines.append('- bg 随机游走是正常行为，只要变化范围有界且平滑即可\n')

    # ── 2. gravity ──
    grav = metrics['gravity']
    lines.append('## 2. 重力估计\n')
    lines.append(f'- 模长均值: {grav["norm_mean"]:.4f} m/s² (理想值: 9.81)')
    lines.append(f'- 模长标准差: {grav["norm_std"]:.4f} m/s²')
    lines.append(f'- |模长 - 9.81| P95: {grav["norm_p95"]:.4f} m/s²')
    lines.append(f'- 后10%模长均值: {grav["tail_norm_mean"]:.4f}, 标准差: {grav["tail_norm_std"]:.4f}')
    lines.append(f'- 是否稳定: {"是" if grav["stable"] else "否"} (后10%标准差 < 0.1)')
    lines.append('')

    if grav['stable'] and abs(grav['tail_norm_mean'] - 9.81) < 0.1:
        lines.append('**结论**: 重力估计稳定且准确。\n')
    elif grav['stable']:
        lines.append('**结论**: 重力稳定但模长偏离 9.81，检查 IMU 加速度计标定或 `acc_cov` 设置。\n')
    else:
        lines.append('**结论**: 重力估计不稳定，系统可能尚未充分收敛。\n')

    lines.append('**参数调优建议**:\n')
    lines.append('- 重力模长偏差大: 检查 IMU 加速度计标定，调整 `acc_cov`')
    lines.append('- 重力振荡: 减小 `b_acc_cov`，增大 `acc_cov`')
    lines.append('- 重力漂移: 增大 `b_acc_cov`\n')

    # ── 3. predict/update 偏差 ──
    diff = metrics['pred_update_diff']
    lines.append('## 3. Predict/Update 偏差\n')
    if diff is not None:
        lines.append('### 位置偏差\n')
        lines.append(f'- 均值: {diff["pos_mean"]:.4f} m')
        lines.append(f'- 标准差: {diff["pos_std"]:.4f} m')
        lines.append(f'- 最大值: {diff["pos_max"]:.4f} m')
        lines.append(f'- P95: {diff["pos_p95"]:.4f} m')
        lines.append('')
        lines.append('### 角度偏差 (SO3 旋转差)\n')
        lines.append(f'- 均值: {diff["angle_mean"]:.4f}°')
        lines.append(f'- 标准差: {diff["angle_std"]:.4f}°')
        lines.append(f'- 最大值: {diff["angle_max"]:.4f}°')
        lines.append(f'- P95: {diff["angle_p95"]:.4f}°')
        lines.append('')

        # 评价
        if diff['pos_p95'] < 0.05 and diff['angle_p95'] < 1.0:
            lines.append('**结论**: 偏差很小，滤波器调参良好。\n')
        elif diff['pos_p95'] < 0.2 and diff['angle_p95'] < 3.0:
            lines.append('**结论**: 偏差中等，系统基本健康，可进一步优化。\n')
        else:
            issues = []
            if diff['pos_p95'] >= 0.2:
                issues.append('位置偏差较大')
            if diff['angle_p95'] >= 3.0:
                issues.append('角度偏差较大')
            lines.append(f'**结论**: {", ".join(issues)}，IMU 预测漂移明显。\n')

        lines.append('**参数调优建议**:\n')
        lines.append('- 位置偏差大: 增大 `gyr_cov`/`acc_cov` 使滤波器更信任激光，或检查 `max_iteration` 收敛性')
        lines.append('- 角度偏差大: 重点调 `gyr_cov`，确认 bg 已收敛')
        lines.append('- 偏差在特定时段突增: 检查是否为退化场景（长走廊、空旷区域）\n')
    else:
        lines.append('无 predict/update 重叠数据。\n')

    # ── 4. velocity 平滑性 ──
    vel = metrics['velocity']
    lines.append('## 4. 速度平滑性\n')
    lines.append(f'- 平均速度: {vel["mean_speed"]:.2f} m/s')
    lines.append(f'- 最大速度: {vel["max_speed"]:.2f} m/s')
    lines.append(f'- 帧间速度变化均值: {vel["jerk_mean"]:.4f} m/s')
    lines.append(f'- 帧间速度变化 P95: {vel["jerk_p95"]:.4f} m/s')
    lines.append(f'- 是否平滑: {"是" if vel["smooth"] else "否"} (P95 < 0.5 m/s)')
    lines.append('')

    if vel['smooth']:
        lines.append('**结论**: 速度平滑，无异常跳变。\n')
    else:
        lines.append('**结论**: 速度存在明显帧间变化，可能存在滤波器不稳定。\n')

    lines.append('**参数调优建议**:\n')
    lines.append('- 速度振荡: 减小 `acc_cov`，增大 `b_acc_cov`')
    lines.append('- 速度漂移: 检查重力估计，调整 `acc_cov`/`b_acc_cov`\n')

    # ── 5. 轨迹跳变 ──
    traj = metrics['trajectory']
    lines.append('## 5. 轨迹跳变检测\n')
    lines.append(f'- 位置跳变 (> 2 m/s 帧间速度): {traj["pos_jump_count"]}')
    if traj['pos_jump_count'] > 0:
        lines.append(f'  - 帧索引: {traj["pos_jump_indices"]}')
    lines.append(f'- 严重跳变 (> 5 m/s): {traj["severe_jump_count"]}')
    lines.append(f'- 是否健康: {"是" if traj["healthy"] else "否"}\n')

    if traj['healthy']:
        lines.append('**结论**: 无轨迹跳变。\n')
    else:
        lines.append('**结论**: 检测到轨迹跳变，检查退化场景或 IMU 数据质量。\n')

    lines.append('**参数调优建议**:\n')
    lines.append('- 位置跳变: 增大 `min_pts`，减小 `filter_size_scan`，检查 `blind` 设置')
    lines.append('- 角度跳变: 检查 `gyr_cov`，确认 bg 已收敛')
    lines.append('- 特征贫乏区域跳变: 增大 `esti_plane_threshold` 或减小 `filter_size_map`\n')

    # ── 6. confidence ──
    conf = metrics['confidence']
    lines.append('## 6. 置信度\n')
    lines.append(f'- 均值: {conf["mean"]:.4f}')
    lines.append(f'- 零值比例: {conf["zero_ratio"]:.2%}')
    lines.append(f'- 低值比例 (< 0.3): {conf["low_ratio"]:.2%}')
    lines.append('')

    if conf['low_ratio'] < 0.05:
        lines.append('**结论**: 置信度整体较高。\n')
    elif conf['low_ratio'] < 0.2:
        lines.append('**结论**: 部分帧置信度较低，可能出现在特征贫乏区域。\n')
    else:
        lines.append('**结论**: 大量帧置信度较低，检查点云质量和匹配情况。\n')

    lines.append('**参数调优建议**:\n')
    lines.append('- 置信度持续偏低: 减小 `filter_size_scan`，增大 `min_pts`，检查 `blind`')
    lines.append('- 特定区域置信度下降: 可能是退化几何，考虑增加约束\n')

    # ── 总结 ──
    lines.append('## 总结\n')
    issues = []
    if not all_bounded:
        issues.append('bg 变化范围过大')
    if not all_smooth:
        issues.append('bg 帧间变化不平滑')
    if not grav['stable']:
        issues.append('重力估计不稳定')
    if diff is not None and (diff['pos_p95'] > 0.2 or diff['angle_p95'] > 3.0):
        issues.append('predict/update 偏差较大')
    if not vel['smooth']:
        issues.append('速度不平滑')
    if not traj['healthy']:
        issues.append('存在轨迹跳变')
    if conf['low_ratio'] > 0.2:
        issues.append('置信度偏低')

    if not issues:
        lines.append('所有健康指标正常，LIO 系统运行良好。\n')
    else:
        lines.append(f'发现问题: {", ".join(issues)}\n')
        lines.append('详见各节参数调优建议。\n')

    path = os.path.join(output_dir, 'analysis.md')
    with open(path, 'w') as f:
        f.write('\n'.join(lines))
    print(f'Saved: {path}')


# ── 主函数 ────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print(f'Usage: {sys.argv[0]} <dump_dir> <output_dir>')
        sys.exit(1)

    dump_dir = sys.argv[1]
    output_dir = sys.argv[2]

    if not os.path.isdir(dump_dir):
        print(f'Error: {dump_dir} is not a directory')
        sys.exit(1)

    predict_path = os.path.join(dump_dir, 'predict_state.txt')
    update_path = os.path.join(dump_dir, 'update_state.txt')

    if not os.path.exists(update_path):
        print(f'Error: {update_path} not found')
        sys.exit(1)

    print(f'Loading update state: {update_path}')
    update = load_state_file(update_path)
    print(f'  {len(update)} frames, duration: {update["timestamp"][-1] - update["timestamp"][0]:.2f}s')

    predict = None
    if os.path.exists(predict_path):
        print(f'Loading predict state: {predict_path}')
        predict = load_state_file(predict_path)
        print(f'  {len(predict)} frames')
    else:
        print(f'Warning: {predict_path} not found, skipping predict/update diff analysis')

    os.makedirs(output_dir, exist_ok=True)

    print('\nGenerating trajectory analysis figure...')
    diff_result = None
    if predict is not None:
        diff_result = plot_trajectory_analysis(update, predict, output_dir)
    else:
        diff_result = plot_trajectory_analysis(update, update, output_dir)

    print('Generating bg convergence figure...')
    plot_bg_convergence(update, output_dir)

    print('Generating health overview figure...')
    plot_health_overview(update, output_dir)

    print('Computing health metrics...')
    metrics = compute_health_metrics(update, diff_result)

    print('Generating analysis report...')
    generate_report(update, predict if predict is not None else update, metrics, output_dir)

    print('\nDone!')


if __name__ == '__main__':
    main()
