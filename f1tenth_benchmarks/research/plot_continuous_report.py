"""Plot saved continuous sessions and write a Chinese Markdown report (no simulation)."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
from PIL import Image
import yaml
from .lap_tracking import ContinuousLapTracker
from .run_continuous_pp import verify_session


def nearest_closed_distance(xy, reference):
    """Unsigned Euclidean distance to closed raceline segments; not Frenet error."""
    edge = np.roll(reference, -1, axis=0)-reference
    square = np.sum(edge**2, axis=1)
    if np.any(square <= 0):
        raise ValueError('degenerate raceline')
    output = []
    for start in range(0, len(xy), 128):
        delta = xy[start:start+128, None, :]-reference[None, :, :]
        fraction = np.clip(np.sum(delta*edge[None, :, :], axis=2)/square, 0, 1)
        residual = delta-fraction[:, :, None]*edge
        output.extend(np.sqrt(np.min(np.sum(residual**2, axis=2), axis=1)))
    return np.asarray(output)


def generate_report(session_dir, report_path, figures_dir):
    session_dir, report_path, figures_dir = map(Path, (session_dir, report_path, figures_dir))
    if report_path.exists() or figures_dir.exists():
        raise FileExistsError('use a new report path and figures directory; outputs are never overwritten')
    verification = verify_session(session_dir)
    summary = json.loads((session_dir/'session.json').read_text())
    rows = [json.loads(line) for line in (session_dir/'samples.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (session_dir/'events.jsonl').read_text().splitlines()]
    raw = np.load(session_dir/f"SimLog_{summary['metadata']['map']}_session.npy")
    meta = summary['metadata']
    tracker = ContinuousLapTracker.from_csv(f"maps/{meta['map']}_centerline.csv")
    if hashlib.sha256(tracker.points.tobytes()).hexdigest() != meta['centreline_sha256']:
        raise ValueError('current centreline differs from recorded reference')
    race_path = Path(f"Data/racelines/{meta['racetrack_set']}/{meta['map']}_raceline.csv")
    if hashlib.sha256(race_path.read_bytes()).hexdigest() != meta['raceline_sha256']:
        raise ValueError('current raceline differs from recorded reference')
    race = np.loadtxt(race_path, delimiter=',', skiprows=1)[:, 1:3]
    xy, speed = raw[:, :2], raw[:, 3]
    time = np.array([r['elapsed_time'] for r in rows])
    unwrapped = np.array([r['tracker_sample']['s_unwrapped'] for r in rows])
    ey = np.array([r['tracker_sample']['projection']['e_y'] for r in rows])
    distance = nearest_closed_distance(xy, race)
    full = [e for e in events if e['kind'] == 'full']
    colors = plt.cm.tab10(np.arange(len(full)) % 10)
    with open(f"maps/{meta['map']}.yaml") as f:
        map_config = yaml.safe_load(f)
    image_path = Path('maps')/map_config['image']
    image = np.flipud(np.asarray(Image.open(image_path).convert('L')))
    origin, resolution = map_config['origin'], map_config['resolution']
    track_closed = np.vstack([tracker.points, tracker.points[0]])
    race_closed = np.vstack([race, race[0]])
    slices = []
    for e in full:
        # Include bracketing actual samples. No fabricated crossing-state point.
        start = max(0, int(np.searchsorted(time, e['lap_start_time']-summary['initialization_time']))-1)
        end = e['post_step_row_index']+1
        slices.append(slice(start, end))
    figures_dir.mkdir(parents=True, exist_ok=False)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})

    def background(ax):
        extent = [0, image.shape[1]*resolution, 0, image.shape[0]*resolution]
        transform = Affine2D().rotate(origin[2]).translate(origin[0], origin[1])+ax.transData
        ax.imshow(image, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=255,
                  alpha=.45, transform=transform, zorder=0)
        ax.plot(track_closed[:, 0], track_closed[:, 1], '--', color='#737373', lw=.8, label='Centreline')
        ax.plot(race_closed[:, 0], race_closed[:, 1], color='#111111', lw=1.1, label='Planned raceline')
        finish = tracker.finish_origin
        line = finish+np.array([-1.1, 1.1])[:, None]*tracker.finish_normal
        ax.plot(line[:, 0], line[:, 1], color='#d62728', lw=2, label='Fixed finish section')
        ax.scatter(*finish, color='#d62728', marker='*', s=65, zorder=6)
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.grid(alpha=.15)
        ax.set_xlim(track_closed[:, 0].min()-3, track_closed[:, 0].max()+3)
        ax.set_ylim(track_closed[:, 1].min()-3, track_closed[:, 1].max()+3)

    def save(fig, name):
        fig.savefig(figures_dir/f'{name}.png', dpi=180, bbox_inches='tight')
        fig.savefig(figures_dir/f'{name}.svg', bbox_inches='tight')
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True,
                             gridspec_kw={'width_ratios': [1.45, 1]})
    for ax in axes:
        background(ax)
        for index, sl in enumerate(slices):
            ax.plot(xy[sl, 0], xy[sl, 1], color=colors[index], lw=1., alpha=.8, label=f'Lap {index+1}')
    axes[0].set_title(f"{meta['map'].upper()}: actual continuous trajectories")
    axes[0].legend(loc='upper right', fontsize=8)
    finish = tracker.finish_origin
    axes[1].set_xlim(finish[0]-2.5, finish[0]+3.5)
    axes[1].set_ylim(finish[1]-1.8, finish[1]+1.8)
    axes[1].set_title('Finish detail: actual post-step samples')
    for index, e in enumerate(full):
        j = e['post_step_row_index']
        axes[1].scatter(xy[j-1:j+1, 0], xy[j-1:j+1, 1], color=colors[index], s=24, zorder=6)
    save(fig, 'route_overview')

    ncols = 3
    nrows = int(np.ceil(len(full)/ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(15, 5*nrows), squeeze=False, constrained_layout=True)
    for index, ax in enumerate(axes.flat):
        if index >= len(full):
            ax.axis('off'); continue
        background(ax)
        sl = slices[index]
        ax.plot(xy[sl, 0], xy[sl, 1], color=colors[index], lw=1.3, label='Actual trajectory')
        ax.set_title(f"Lap {index+1} | {full[index]['lap_label']} | {full[index]['duration']:.4f} s")
        if index == 0:
            ax.legend(fontsize=7, loc='upper right')
    save(fig, 'routes_by_lap')

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    durations = np.array([e['duration'] for e in full])
    axes[0, 0].bar(np.arange(1, len(full)+1), durations, color=colors)
    axes[0, 0].set_xlabel('Full lap'); axes[0, 0].set_ylabel('Estimated lap time [s]')
    axes[0, 0].set_ylim(0, max(durations)*1.13)
    for index, duration in enumerate(durations):
        axes[0, 0].text(index+1, duration+.35, f'{duration:.4f}', ha='center')
    axes[0, 1].plot(time, unwrapped/tracker.L, color='#356b9a')
    axes[0, 1].set_ylabel('Unwrapped s / L'); axes[0, 1].set_xlabel('Session elapsed time [s]')
    axes[1, 0].plot(time, speed, lw=.8, color='#356b9a')
    axes[1, 0].set_ylabel('Physical speed [m/s]'); axes[1, 0].set_xlabel('Session elapsed time [s]')
    axes[1, 1].plot(time, ey, lw=.8, label='Signed centreline e_y')
    axes[1, 1].plot(time, distance, lw=.8, alpha=.8, label='Unsigned nearest raceline distance')
    axes[1, 1].set_ylabel('Distance [m]'); axes[1, 1].set_xlabel('Session elapsed time [s]')
    axes[1, 1].legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    for ax in [axes[0, 1], axes[1, 0], axes[1, 1]]:
        for e in full:
            ax.axvline(e['crossing_time_estimate']-summary['initialization_time'], color='#888888', alpha=.35, lw=.6)
    save(fig, 'session_diagnostics')

    flying = np.array([e['duration'] for e in full if e['lap_label'] == 'flying'])
    manifest = json.loads((session_dir/'protected_hashes.json').read_text())
    changed = [p for p, digest in manifest['before'].items()
               if not Path(p).is_file() or hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest]
    stats = dict(verification=verification, max_abs_centreline_e_y=float(np.max(np.abs(ey))),
                 raceline_nearest_distance_mean=float(distance.mean()),
                 raceline_nearest_distance_p95=float(np.percentile(distance, 95)),
                 raceline_nearest_distance_max=float(distance.max()),
                 speed_min=float(speed.min()), speed_max=float(speed.max()),
                 negative_s_increments=int(np.sum(np.diff(unwrapped) < -1e-9)),
                 projection_rejections=sum(not r['projection_valid'] for r in rows),
                 flying_mean=None if not len(flying) else float(flying.mean()),
                 flying_std_population=None if not len(flying) else float(flying.std()),
                 flying_range=None if not len(flying) else float(np.ptp(flying)),
                 final_sample_after_crossing_seconds=summary['simulation_time']-full[-1]['crossing_time_estimate'],
                 protected_files_changed=changed,
                 input_hashes={name: hashlib.sha256((session_dir/name).read_bytes()).hexdigest()
                               for name in ['session.json','events.jsonl','samples.jsonl',f"SimLog_{meta['map']}_session.npy"]},
                 map_image_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest())
    if changed:
        raise RuntimeError(f'protected inputs changed: {changed}')
    with (figures_dir/'analysis.json').open('x') as f:
        json.dump(stats, f, indent=2)
    def link(path):
        return Path(os.path.relpath(path, report_path.parent)).as_posix()
    lines = [f'# {session_dir.name}：連續圈路線圖與結果分析', '',
             f"分析日期：{datetime.now(timezone.utc).astimezone().date()}；來源：`{session_dir}`。", '',
             f"本次完成 **{summary['completed_full_laps']} 個完整圈**，終止原因 `{summary['end_reason']}`。"
             f"只 reset {summary['reset_count']} 次，partial={summary['partial_event_count']}，"
             f"collision={summary['collision']}，projection rejections={stats['projection_rejections']}。", '',
             '## 路線圖', '',
             '以 ESP 地圖影像、resolution 與 origin 對齊世界座標（m）。灰色虛線為固定中心線，黑線為 mu60_closed raceline；彩色線為各圈實際 post-step XY。紅星及紅線表示固定起終點與法向截面。', '',
             f"![全場軌跡與終點放大]({link(figures_dir/'route_overview.png')})", '',
             '總圖中各圈高度重疊，因此另外提供逐圈圖。跨線前後圓點均為實際樣本，沒有產生／假造精確 crossing state；每圈切片保留邊界前後樣本。', '',
             f"![各圈實際路線]({link(figures_dir/'routes_by_lap.png')})", '',
             '## 圈時間與穩定性', '', '| 完整圈 | 類型 | 圈時間估計（s） |', '|---|---|---:|']
    for index, e in enumerate(full):
        lines.append(f"| {index+1} | {e['lap_label']} | {e['duration']:.6f} |")
    if len(flying):
        lines += ['', f"Flying laps 平均 **{flying.mean():.6f} s**，母體標準差 {flying.std():.6f} s，"
                      f"最快與最慢差 {np.ptp(flying):.6f} s（平均的 {100*np.ptp(flying)/flying.mean():.4f}%）。",
                  '起步圈與 flying laps 分開比較；本次結果可重現，不代表其他配置或長期運行的普遍穩定性。']
    lines += ['', '## 進度、速度與路線偏移', '',
              f"![全場診斷]({link(figures_dir/'session_diagnostics.png')})", '',
              f"- 樣本數：{summary['sample_count']}；control steps：{summary['control_steps']}；預設 outer tick：0.04 s。",
              f"- 暖機後 session 時間：{summary['elapsed_time']:.6f} s；物理 simulation time：{summary['simulation_time']:.6f} s；暖機時間：{summary['initialization_time']:.2f} s。",
              f"- 速度範圍：{speed.min():.4f}–{speed.max():.4f} m/s；unwrapped s 負增量數：{stats['negative_s_increments']}。",
              f"- 中心線最大 |e_y|：{stats['max_abs_centreline_e_y']:.6f} m。",
              f"- 閉合 raceline 最近線段的無號幾何距離：平均 {distance.mean():.6f} m，P95 {np.percentile(distance,95):.6f} m，最大 {distance.max():.6f} m。",
              '', '中心線偏移不等於 raceline 追蹤誤差：中心線用於 Frenet 進度，raceline 用於規劃。此處新增的 raceline 距離是全域最近線段的 Euclidean 幾何距離，沒有沿程匹配，不能當作控制器穩定性保證或碰撞餘裕。', '',
              '## 已執行的驗證與限制', '',
              '- verify_session 通過：完整圈數、partial 數、一次 reset、post-step NPY、事件插值與前後樣本時間、N 圈 crossing 當步終止。',
              '- 原 DynamicsSimulator 獨立重播每個樣本（state atol=1e-12），確認七維 state 與 steering buffer 持續延續，crossing 沒有 reset；scan 形狀及有限值檢查通過。',
              '- 原受保護檔案目前 hashes 未變；中心線與 raceline hashes 和該 session metadata 一致。',
              f"- 最終 physical sample 晚於 crossing estimate 約 {stats['final_sample_after_crossing_seconds']*1000:.3f} ms，保留实际樣本，不 rewind dynamics。",
              '- 圈時間為沿程線性插值估計，並非精確 crossing 時刻；只驗收本 ESP/config。原 sampled 四角碰撞模型與 Stage 2 取樣／投影限制仍適用。', '',
              '## 產物與重現方式', '',
              f"- [原圈結果 CSV]({link(session_dir/'lap_results.csv')})", f"- [原 session metadata]({link(session_dir/'session.json')})",
              f"- [本次分析數值與輸入 hashes]({link(figures_dir/'analysis.json')})", '',
              '所有圖皆另存 PNG（閱讀）與 SVG（向量匯出）。本次只讀取已保存 simulation，不重新賽車、不修改原始結果。產生器拒絕覆寫已有 report／figures 目錄。', '',
              '在已開啟的 Docker 容器 `/workspace` 執行；若重新產生，請換全新 report／figures 名稱：', '', '```bash',
              'python -B -m f1tenth_benchmarks.research.plot_continuous_report \\',
              f'  --session-dir {session_dir} \\', f'  --report {report_path} \\', f'  --figures-dir {figures_dir}', '```', '']
    with report_path.open('x') as f:
        f.write('\n'.join(lines))
    return dict(report=str(report_path), figures=str(figures_dir), statistics=stats)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--figures-dir', required=True)
    args = parser.parse_args()
    print(json.dumps(generate_report(args.session_dir, args.report, args.figures_dir), indent=2))


if __name__ == '__main__':
    main()
