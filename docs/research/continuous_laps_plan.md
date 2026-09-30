# Continuous laps：第一階段計畫

更新日期：2026-09-30

## 目標與範圍

建立 research-specific continuous laps，保留原始 benchmark 行為。使用 ESP、mu60 幾何規劃方案與現有 GlobalPurePursuit。只做一次 initial reset；跨 finish 時記錄圈次而不重設車輛。保留七維 state `[x, y, steering, velocity, yaw, yaw_rate, slip]` 與 steering delay buffer。Collision 終止整場；完成指定 N 圈後由 runner 結束。Timeout 獨立標記，不算完成圈。

本階段不實作 ROS2、opponent、overtaking、五候選軌跡或 local-path controller。Runner 保留 `planner.plan(observation)` 接口，避免將 planner 固定在 simulator 內部。

## 保存的 baseline

- Repository：`/data/f1tenth/sim/f1tenth_benchmarks`
- 原始分支：`overtaking-planner`
- Map：`esp`；planner：`GlobalPurePursuit`；planner name：`ESP_GlobalPP`；test ID：`esp_pp_test`
- Raceline：`Data/racelines/mu60/esp_raceline.csv`
- 生成參數：`Data/raceline_data/mu60/params.yaml`，mu = 0.60
- 舊 planned time：40.07378694588353 s（對話中的 generator 輸出；未找到保存此數值的文字 log）
- Simulation results：`Logs/ESP_GlobalPP/Results_ESP_GlobalPP.csv`
- 保存結果：Lap 0、Time 41.5200 s、Steps 1038、Progress 0.9967、LapComplete True、Collision False
- Raw log：`Logs/ESP_GlobalPP/RawData_esp_pp_test/SimLog_esp_0.npy`，shape `(1038, 10)`
- 預設 outer step：4 × 0.01 s = 0.04 s，即 25 Hz 模擬時間頻率。

## Source 調查結論

- `run_scripts/run_functions.py:9` 的 `simulate_laps()` 每個 episode 都 reset；目前沒有真正 continuous-lap mode。
- `simulator/f1tenth_sim.py:68` 的 step 將 lap complete 與 collision 合併成 done，並立即保存。不能只忽略 done 或在 `super().step()` 後修改 done。
- `f1tenth_sim.py:101` 使用 progress 門檻，不是 crossing detector；250 秒也回傳 lap complete。
- `f1tenth_sim.py:134` 的 reset 清 physical state，執行一次零 action step；total_steps 不歸零。
- `simulator/dynamics_simulator.py:102` 的 reset 也清 steering buffer。
- `classic_racing/RaceTrackGenerator.py:74` 使用 closed minimum-curvature geometry；第 91 行速度求解使用 closed=False、v_start=max_speed。
- 舊時間計算的段長不包含最後一點回第一點。ESP 舊 CSV 有 1153 點，closure distance 約 0.19985647724542582 m；首末速度皆 8 m/s。
- `trajectory_planning_helpers/trajectory_planning_helpers/calc_vel_profile.py` 已提供 closed solver；closed 模式要求 N 點對應 N 個段長。
- 原詳細 logger 保存 step 前 state；finish 更新後 state 需另外記錄。原 save_history 沒有清 scans，research logger 應獨立管理。

上述行號是調查時的位置；實作前重新核對。

## 分段實作與驗收

### 階段 1：閉合速度規劃與結果比較

允許重新執行最小曲率計算，保持同一方案與可追溯設定。另存結果，不覆寫 baseline。比較三組：

1. 舊 mu60：保存的基準。
2. 固定舊幾何 + closed speed：隔離閉合速度條件的影響。
3. 重新計算最小曲率 + closed speed：量化重新生成的幾何與速度差異。

預計新增 `research/closed_velocity_profile.py`，使用既有 helper 求解器，補 closure segment、使用 closed=True，不指定 open v_start/v_end。需要重新算幾何時沿用原 minimum-curvature 算法，不修改原生成器；注意避免其原始輸出路徑覆寫舊資料。

驗收：

- 固定幾何組的點數、順序、s/x/y/heading/curvature 與舊 mu60 完全一致。
- 重算組比較點数、位置、heading、curvature、圈長；不要求逐值完全相同。若點數或取樣不同，以共同弧長取樣比較，yaw 差使用角度 wrap。
- 比較 speed、acceleration、跨接縫可行性與 planned time，保存設定與差異報告。
- N 點對應 N 個段長；完整 planned lap time 包含 closure segment，且只計一次。
- 不要求第一筆和最後一筆速度相等，因為它們是不同位置；要求跨閉合段符合求解器限制。
- 所有數值有效，舊 mu60 與 baseline logs 未被覆寫。
- 不要求新 planned time 一定更快；minimum curvature 不等於直接 minimum lap time。

新 raceline set 命名待區分兩組後固定，例如 `mu60_closed` 與 `mu60_closed_regenerated`，不可讓兩組輸出互相覆寫。

Stage 1 執行補充（2026-09-30）：上述名稱已固定。新 CSV 使用 comment header，配合原 loader 的 skiprows=1；固定組前五欄讀回 exact equality。加速度使用 N+1 extended speeds（尾端接 v0）和 N 個段長、eq_length_output=False。幾何重算不建立原 generator，沿用其數值 helper 呼叫，min-curve 隔離保存；effective kappa bound 保留原呼叫的 1。

已完成計算與比較，但 strict combined-force 驗收未通過：raw closed helper output 最大離散殘差 0.06555353582364232 m/s²（162 段超過 1e-6），closure 通過；舊 profile 同檢查無超差。不得提高容差掩蓋問題。Stage 1 尚未完全完成，下一步先診斷 closed helper 的前後向離散一致性；若採 research-only 保守修正，保留 raw 結果並另存新版本。Stage 2 暫不開始。

### 階段 2：跨圈偵測與計時

新增 `research/lap_tracking.py`：`ContinuousLapTracker`。

管理 previous/current progress、previous pose、固定 finish reference、crossing armed、lap start time、last lap time、completed lap count。使用原始 centre_line_progress，不沿用會被清零的 lap_progress。正向 crossing 需配合 rearm 與足夠圈距，避免投影抖動誤計。

以人工 pose/progress/time 序列驗證：正常跨圈一次、起步附近不計、抖動不重複、倒車不計、兩圈獨立計時、不同起點的首次 partial segment。

### 階段 3：Continuous simulator

新增 `research/continuous_sim.py`：`F1TenthSim_Continuous(F1TenthSim_TrueLocation)`。

沿用 dynamics/map/scan/collision；override 必要 lifecycle，保留每步四次 dynamics 子步顺序。跨圈不 terminal；collision terminal。時間與 steps 全場累加，lap 時間用差值。

驗收：initial reset 一次；crossing 不呼叫 dynamics reset、不清七維 state 或 steering buffer；下一 dynamics update 承接 crossing 後 state；超過 250 秒不誤計圈。先短步進再完整圈。

初始計時需明確固定：建議保留初始化零輸入 step，以初始化完成時作 research 計時原點。固定 finish 線與 initial pose 的關係需核對；不同位置出發的第一次 crossing 若未滿圈，記為 partial。首圈 standing-start 與後續 flying laps 分開標記。

### 階段 4：Research logging

新增 `research/continuous_logging.py`：`ContinuousLapLogger`。

保存 session 設定/結束原因、lap results、simulation state/action/progress、crossing post-step 七維 state。可保留十欄 NPY 作原 plotting 相容輸出，時間/event 另存。使用新 research planner/test ID，避免重複 EntryID 與覆寫 baseline。

驗收：每 crossing 一筆 lap；time=end-start；結果數等於完成圈數；partial 獨立；collision/timeout/requested_laps_reached 有不同原因；explicit finalize，防止 destructor 重複 flush；scan buffers 正確清理。

### 階段 5：Runner 整合與 ESP 驗收

新增 `research/run_continuous_pp.py`：`run_continuous_laps()`、`run_esp_continuous_pp()`。建立 planner/set_map，initial reset 一次，每 control tick plan/step；collision、N 圈完成或研究 timeout 後 finalize。

驗收：ESP 三個完整 continuous laps，記錄起步圈與兩個 flying laps；跨圈無 reset；達 N 圈當步結束；collision 可提前結束；全部結果可驗證。不得把 simulated time 等於 planned time 作通過條件。

Baseline 回歸：原 GlobalPurePursuit runner 仍可原樣執行，另用隔離輸出 ID 做驗證，不能覆寫昨天結果。

## 預計新增檔案

```text
f1tenth_benchmarks/research/
├── __init__.py
├── closed_velocity_profile.py
├── lap_tracking.py
├── continuous_sim.py
├── continuous_logging.py
└── run_continuous_pp.py

tests/research/
└── 依各階段新增有意義的數值與 lifecycle 檢查
```

原 `RaceTrackGenerator.py`、`GlobalPurePursuit.py`、simulator/dynamics/utils、benchmark runners、參數、mu60 與昨日 logs 不需修改。

## 未來接口

Runner 管 planning 調度，simulator 管 dynamics/lifecycle。未来五候選軌跡選路與 cached path 另加；lap event 不清 planner cache 或 opponent state。25 Hz control 每兩步 planning 為 12.5 Hz；精確平均 10 Hz 可另用 simulation-time scheduler。這些不在第一階段實作。

## 執行與接續方式

按階段 1→5 推進，每批通過後更新 `continuous_laps_progress.md`，記錄 commands、結果、產物路徑與尚未解決的決策。每次接續先讀兩份文件、檢查 Git 與實際產物，不能只依文件勾選推定完成。這份文件的建立不代表已執行計算或 simulation。


## Stage 1 正式完成（2026-09-30）

前述 raw helper 未通過的紀錄為歷史狀態。使用者授權的 research-only corrected solver 已修兩處索引問題，重新生成兩組與四組比較報告；12 regression tests PASS。最大 combined residual 2.6645352591003757e-14 m/s²（tol=1e-9），零違規；不同起點速度一致、closure accounting、幾何 preservation、baseline/upstream hash 保護全部 PASS。完整時間 40.09876900553927 s。Stage 1 完成，Stage 2 尚未開始。詳細 commands、產物與接續見 progress。
