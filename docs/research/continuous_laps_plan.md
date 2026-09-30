# Continuous laps：實作計畫與研究方向

更新日期：2026-09-30

## 目前完成狀態與接續方向

Stage 1 閉合速度規劃與 Stage 2 獨立 Frenet tracker 已完成驗證；Stage 3 simulator、Stage 4 logger、Stage 5 runner 尚未實作。最新 research 測試為 24 項 PASS（Stage 1、2 各 12 項）。尚未執行完整 continuous-lap vehicle simulation，因此目前不能宣稱已達成連續三圈驗收。

接續工作依序為：

1. **Stage 3：最小 simulator 整合。** 初始化完成後建立 tracker，之後將每個實際 post-step pose、time、signed velocity 傳入 update。先驗證短步進與跨 finish 的 lifecycle：不 reset 車輛、不清 steering delay，collision／timeout 與完成圈分開；tracker 異常需明確處理。
2. **Stage 4：保存可追溯資料。** 分開保存 full／partial events、crossing 時間估計與實際 physical samples；記錄固定中心線 reference、tracker 配置、投影異常及 recovery epoch，確認 finalize 不重複寫入。
3. **Stage 5：runner 與 ESP 驗收。** 以 completed_full_laps 達 N 為完成條件；中途出發的 partial 不占 N。完成整合與短步進驗證後，才執行 ESP 三個完整圈及隔離輸出的 baseline 回歸。

下一階段先驗證 tracker 在真實 dynamics 下的取樣頻率、局部搜尋範圍及 Frenet 可行性配置，不將人工中心線投影測試視為 simulator 驗收。LMPC、ROS2、opponent、overtaking 與候選軌跡仍屬後續研究範圍。各階段實際 commands、產物及限制見 [進度紀錄](continuous_laps_progress.md)。

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

歷史紀錄（已由末尾 Stage 1 正式 PASS 取代）：已完成計算與比較，但 strict combined-force 驗收未通過：raw closed helper output 最大離散殘差 0.06555353582364232 m/s²（162 段超過 1e-6），closure 通過；舊 profile 同檢查無超差。不得提高容差掩蓋問題。Stage 1 尚未完全完成，下一步先診斷 closed helper 的前後向離散一致性；若採 research-only 保守修正，保留 raw 結果並另存新版本。Stage 2 暫不開始。

### 階段 2：固定 Frenet reference、跨圈偵測與計時（已實作）

獨立模組 `f1tenth_benchmarks/research/lap_tracking.py` 提供 `ContinuousLapTracker`、`TrackerConfig`、immutable Projection/Sample/LapEvent/UpdateResult；不改原 benchmark，也不操作 dynamics。

論文依據：Learning How to Autonomously Race a Car: A Predictive Control Approach，Section II 式 (3) 用中心線圈長 L 定義終點集合 s ≥ L；IV-A 使用 s、e_y、e_psi；V 延伸資料到終點之後以支援連續賽車。本 workspace 未找到 PDF，依使用者提供定義實作。wrapped/unwrapped、crossing 防抖、插值與恢復是本專案工程設計，不是論文完整實作的重現。

**Reference 與 source 調查**

- 使用 `maps/esp_centerline.csv` 原始完整中心線 XY；constructor 接受 Nx2 或 `from_csv()` 讀有／無 header 的 CSV。固定第一點為 s=0；initial pose 不移動 reference。幾何 finish 為第一點通過 outgoing tangent 的法向截面，公開 finish_origin/tangent/normal。多邊形接縫可能有切向不連續；主要判據仍是連續 s 門檻，不以 heading 推定運動方向。
- 尾點若與首點距離 ≤1e-9 m，先去掉重複尾點；其餘零長度段拒絕。每條線段包含 last→first，closure 只計一次。
- ESP 有 1183 點、無重複終點；L=237.3299333067795 m，closure=0.20049618714327286 m。smooth ESP 為不同 reference（1180 點，L=235.93096368210848 m），Stage 1 raceline L=230.45281503736692 m，均不得混用。
- 原 `TrackLine.init_path()` 的 s_path 只累加相鄰點，不包含 closure；`calculate_progress_m()` 用 nearest waypoint 選 spline 取樣區間，端點有回傳 s_path[-1] 的捷徑，並非閉合線段弧長投影。spline 參數亦未保證為弧長，因此不重用 progress×L。原 CentreLine 的 skiprows=1 會跳過 ESP 無 header CSV 第一個數值點；此階段不修改原 loader。

**座標、投影與有效性**

- s_wrapped ∈ [0,L)，由閉合線段正交投影與段內 fraction 計算；e_y 為段切線左側正值，e_psi 是可選 yaw 診斷。
- s_unwrapped 由最短 signed wrapped increment 展開；99.8→0.2 展開為 99.8→100.2（L=100），倒車可下降。它是沿中心線的座標，不是行駛里程，也不是 completed count。
- 首次全域搜尋，之後只搜尋前次 segment ±local_segments（預設 30，循環索引）。候選投影幾何距離接近（預設 0.02 m）但沿程相隔超過 1 m，拒絕為 ambiguous_projection；局部搜尋保留既有 branch，不會全域跳至較近的其他賽段。未知初始 branch 的歧義必須由 caller 處理。
- `bound = frenet_factor * max(abs(previous_speed), abs(current_speed)) * dt + increment_margin`。缺速度用 max_speed；預設 factor=2、margin=0.1 m、max_speed=20 m/s、max_dt=0.2 s、最大投影距離=2 m。拒絕 |ds|>bound、bound≥L/2、dt 超限及非法／非遞增時間。max_dt 比較允許 1e-12 s 浮點誤差。
- 假設局部投影唯一，曲率與側向偏移不使 Frenet 放大因子（平滑線近似 1/|1−κe_y|）超過配置，且 sample endpoint speeds 能涵蓋該 interval 運動速度。這不是車速×dt 的恆等式，也不是普遍可行性證明。高速、急彎、較大偏移需要另行校準配置，不能靠放寬門檻消除歧義。
- 拒絕後 previous/current、進度與圈數保留，invalid_reason 與 recovery_required latch；update 不再產生事件。caller 使用較晚新樣本明確 `recover()` 做全域定位；丟棄中斷圈計時，epoch++，重新建立 s 座標、下一 L 門檻及 partial。已有完整圈 count/events 保留；不跨 epoch 推算缺失圈數。恢復本身沒有 event，不補計圈。

**初始化、partial 與 crossing**

- 起终點初始化条件：min(s_wrapped,L−s_wrapped) ≤0.05 m 且投影距離 ≤0.2 m，可設定。接縫前側小負 s 以負座標初始化，接縫後側小正 s 保留；next_finish_s=L，不立即計圈。這是容差內近似起點，初始時間開始完整圈計時。
- 只有上述條件成立，首次 crossing 才為完整起步圈；已知 |initial speed|≤0.05 m/s 標 standing_start，未知速度標 initial_speed_unknown，其餘 moving_start。後續完整圈標 flying。
- 其餘初始化為 partial interval。例如 s=30→100 記 partial，不增加 completed_full_laps；100→200 才是第一完整圈。partial duration 與 full duration 分開，不能以 floor(s/L) 算完整圈數。
- 正向 crossing 要求有效投影、ds>0、armed 且 s_prev<next_finish_s≤s_curr。initial full interval 在距初始化 s 前進 rearm_distance（預設 1 m）後 armed；initial partial 已 armed，允許從終點前很近的位置開始。
- crossing 後 next_finish_s+=L，disarm，以該 crossing 門檻為 arm_origin；前進至 arm_origin+rearm_distance 再 armed。已觸發門檻不回退，終點抖動及倒車再前進不能重複觸發舊門檻。投影抖動幅度與錯 branch 必須由有效性假設控制。

**時間與介面**

- `alpha=(next_finish_s−s_prev)/(s_curr−s_prev)`，檢查 dt>0、分母>0、alpha∈[0,1]；`t_cross=t_prev+alpha*dt` 是沿程線性插值估計，不是精確 crossing 時刻。完整圈 duration 為有效相鄰計時邊界差；起步圈的前邊界是初始化時間。
- event 保存 kind、lap_label、completed_full_laps、finish_s、crossing_time_estimate、lap_start_time、duration、alpha、前後 Sample（time/index/projection/s_unwrapped/speed）、epoch、timing_method。實際 post-step 七維 state 由未來 simulator/logger 保留，不能稱為精確 crossing state。
- 最小接口：初始化後 `tracker.update(pose=[x,y,yaw], time=simulation_time, speed=signed_velocity)`；回傳 valid/reason/sample/event。管理 previous/current、next_finish_s、armed、lap_start_time、completed_full_laps、events 與 recovery_required。不 reset 車輛、不清 steering delay、不改七維 state。
- 低頻取樣、單步可能超過半圈或多圈不能可靠 unwrap；明確拒絕 max_dt／bound≥L/2，不從首尾位置猜漏圈。若 caller 提供錯誤速度界限，首尾 alias 仍可能無法偵測；未知交會 branch、局部 window 不足、離開 reference 的運動不保證可用。

驗收：人工序列與實際 ESP 中心線重取樣（不是 vehicle simulation）測試閉合長度、接縫、初始化、倒車、抖動、兩圈各自計時、partial、側向偏移、事件插值、異常凍結／恢復、時間與取樣限制、歧義與局部 branch 延續。

### 階段 3：Continuous simulator

新增 `research/continuous_sim.py`：`F1TenthSim_Continuous(F1TenthSim_TrueLocation)`。

沿用 dynamics/map/scan/collision；override 必要 lifecycle，保留每步四次 dynamics 子步顺序。跨圈不 terminal；collision terminal。時間與 steps 全場累加；以 post-step pose/time/signed speed 呼叫 Stage 2 tracker，從 event 的插值邊界計算圈時間。invalid 結果須記錄並由 runner 明確決定停止或 recover，不得自動補圈。

驗收：initial reset 一次；crossing 不呼叫 dynamics reset、不清七維 state 或 steering buffer；下一 dynamics update 承接 crossing 後 state；超過 250 秒不誤計圈。先短步進再完整圈。

初始化零輸入 step 完成後的實際樣本作 tracker 初始時間；中心線第一點的固定 finish 不隨 initial pose 移動。依 Stage 2 容差判定 initial full／partial；只有已知初始速度符合條件才標 standing_start，否則 moving_start／initial_speed_unknown。partial 後由 crossing estimate 開始完整圈計時。

### 階段 4：Research logging

新增 `research/continuous_logging.py`：`ContinuousLapLogger`。

保存 session 設定／結束原因與 reference/config，full/partial events，simulation state/action、s_wrapped/s_unwrapped、projection validity/reason、epoch、sample index/time；保留 crossing estimate、alpha、前後 sample time/index/projection 與實際 post-step 七維 state。post-step state 是物理樣本，不能標成精確 crossing state。可保留十欄 NPY 作原 plotting 相容輸出，時間/event 另存。使用新 research planner/test ID，避免重複 EntryID 與覆寫 baseline。

驗收：每有效 crossing 一筆 event；duration=end estimate−start boundary；full results 數等於 completed_full_laps，partial 獨立且不計入 N；collision/timeout/requested_laps_reached 有不同原因；explicit finalize，防止 destructor 重複 flush；scan buffers 正確清理。

### 階段 5：Runner 整合與 ESP 驗收

新增 `research/run_continuous_pp.py`：`run_continuous_laps()`、`run_esp_continuous_pp()`。建立 planner/set_map，initial reset 一次，每 control tick plan/step；collision、tracker.completed_full_laps 達 N 或研究 timeout 後 finalize；partial 不計入 N，不能以 crossing 數或 floor(s_unwrapped/L) 結束。

驗收：ESP N=3 個完整 continuous laps；符合起點初始化条件時記錄 initial full 圈（依速度分類）與兩個 flying laps，中途出發則先獨立 partial，再三個完整圈；跨圈無 reset；達 N 圈當步結束；collision 可提前結束；全部結果可驗證。不得把 simulated time 等於 planned time 作通過條件。

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

前述 raw helper 未通過的紀錄為歷史狀態。使用者授權的 research-only corrected solver 已修兩處索引問題，重新生成兩組與四組比較報告；12 regression tests PASS。最大 combined residual 2.6645352591003757e-14 m/s²（tol=1e-9），零違規；不同起點速度一致、closure accounting、幾何 preservation、baseline/upstream hash 保護全部 PASS。完整時間 40.09876900553927 s。Stage 1 完成；本次 Stage 2 已實作並驗證，詳細 commands、產物與接續見 progress。
