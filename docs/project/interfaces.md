# 介面與座標定義

本文件定義現行介面；完成狀態、驗收數值與限制見[完成總結](status.md)。

## 程式入口與用途

| 模組／入口 | 輸入 → 輸出 | 使用位置 |
|---|---|---|
| build_external_map | 無既有中線的閉合影像／YAML、方向 → 幾何及來源紀錄 | 外部地圖準備階段 |
| build_open_map | 開放走廊影像／YAML、start-hint → 有限中線、兩側邊界與 manifest | 開放地圖準備階段 |
| build_map | 影像／YAML、anchor CSV → 版本化中線、邊界、曲線與 manifest | 地圖準備階段 |
| load_geometry | 幾何目錄、原 YAML → TrackGeometry；核對來源與檔案雜湊 | 啟動時載入 |
| TrackGeometry.to_frenet | map x/y、可選 yaw → s_wrapped、d、可選 e_psi、有效性 | 單點轉換核心 |
| TrackGeometry.to_cartesian | s、d、可選 e_psi → map x/y、可選 yaw、有效性 | 反向轉換核心 |
| TrackGeometry.reference／boundaries | s → 中線位置／方向／曲率，或逐截面左右邊界 | 路線與地圖查詢 |
| FrenetTracker.update／update_tf | 連續 pose／TF、時間與版本 → 連續 s、d、圈數與有效性 | 車輛定位追蹤 |
| DomainAtlas.query／section | s/d → 精確有效性；s → 道路邊界及必要 J 範圍 | 域查詢與檢視 |
| section_ranges.query_section | s → 道路範圍、Frenet 採樣區間、原因與驗證程度 | 六張圖共用 |
| MapInspector.query_pixel／query_sd | 連續像素 c/v 或 s/d → map、投影、候選／邊界及重建診斷 | 本機點選工具 |
| run_continuous_laps | sim、planner、圈數、輸出目錄 → 行車紀錄與摘要 | 連續圈研究 |

核心雙向轉換在 `f1tenth_benchmarks/research/core/geometry/frenet.py`。
tracker 重用候選投影並加入歷史判斷，domain 重用精確反轉換；兩者不是另外一套
座標公式。批次驗證程式不需要在每個控制步驟執行。

## 座標定義

- map x/y 與 s/d 單位為 m；角度 rad；曲率 1/m。
- s 是沿中線的實際弧長；固定起點／正向保存在幾何版本中。閉合圖使用週期曲線，開放圖使用有限曲線。
- 閉合圖 s_wrapped 在 [0,L)；開放圖保留欄位名稱，但值在 [0,L]，不取模或外插。左側 d 為正，右側為負，左右寬度不必相等。
- e_psi 是車輛 yaw 減去中線方向，包至 [-pi,pi)；沒有 yaw 就不推定航向。
- 反向公式為 `xy = reference(s) + d * normal(s)`；normal 指向中線正向的左側。
- FrenetTracker 的 s_unwrapped 可跨起點連續，倒退時可減少；wrap_count 是本次初始化後跨接縫的有號圈數，不等於競賽完整圈計數。
- 新中線的 s、原折線圈追蹤的 s、raceline 的 s 各有自己的參考線，不可互換。

ESP 目前候選長度236.907497773 m，s=0 約(0.0769390711,−0.0125069472)，
正向為順時針；每次改曲線生成新 geometry_id，不能默默續用舊進度。

影像像素中心的 c/v 由左上起算。令 r 為 resolution，H 為圖高：
`q=((c+0.5)*r,(H-v-0.5)*r)`，再用 YAML origin 的旋轉和平移取得 map x/y。
occupancy 使用 YAML 的 negate／門檻，unknown 不視為可行駛。

## 結果有效性

to_frenet 成功時提供 s_wrapped、d、projection_xy、psi_reference、kappa、
可選 e_psi，以及 geometry_id／frame_id／valid／reason。
無效單點不提供假有效 s/d；to_cartesian 可保留無效重建點供檢視，但 valid=false。

判斷包含 occupancy、道路邊界、投影分支與 `J=1-kappa*d≥0.2`。
常見原因為 occupied、unknown、outside_map、outside_track、ambiguous_projection、
singular_frenet；J 低於工程餘裕不一定代表精確 J=0。
單點的 s_hint／search_window_m 只能限制窗口，不能掩蓋全域歧義。

valid／inside_valid_domain 只表示本次局部查詢通過。
domain_verified=false 表示尚未完成全域驗收，planning_allowed=false 表示
候選尚未批准規劃使用；load_geometry 目前必須指定 inspect=True。
DomainAtlas 的 sampled_runs 不認證採樣間的連續區間；query 每次精確檢查。
section 的必要 J 範圍也不保證投影唯一。boundaries 每次由原 occupancy 格子射線
求值，CSV 邊界插值尚未批准。原道路範圍保留，不被 Frenet 拒絕區刪除。
邊界保存的 s/d 代表來源截面，無效時仍保留 x/y 與來源標記，不能以另一個最近
投影覆蓋來源。D 的診斷驗收與規劃批准分開；完整插值／連續域認證不列為本次 D 必做。
`/api/map` 可附 issues 與 geometry_audit；issue 含分類、來源樣本、採樣 s 範圍、
是否跨接縫、J 摘要與檢視像素位置。continuous_verified=false 表示區段只作診斷。
`acceptance.json` 的 stage_d_diagnostics_complete=true 表示新目標結案；
舊密集報告 stage_d_complete=false 保留，不代表目前診斷工作未完成。

新增 `section_ranges.query_section(geometry, s)` 與
`GET /api/section?map=INDEX&s=S`：回傳實際道路 `road_range`、零個／多個
`frenet_ranges`、附拒絕原因的 `limited_ranges`、逐點 `samples` 與驗證程度。
道路範圍來自原 occupancy 精確射線；Frenet 區間來自逐點轉換採樣，
`verification_level=sampled_requires_exact_query`、`continuous_verified=false`。
預設 d 間距不超過0.025 m及半個像素，已觀察到的有效／拒絕轉折加密至0.0001 m。
此解析度不保證發現採樣之間的窄小拒絕區，也不允許直接插值當成有效性判定。
necessary_d_min/max 只檢查 J 的必要條件，不保證唯一投影；sampled_runs 不認證
採樣間的區間。實際候選必須逐點精確確認，後續規劃器再於 x/y 檢查車身與動態。

## 點選位置如何轉成 Frenet

實作入口為 `frenet.py` 的 `_candidates`、`_check_candidate` 與 `to_frenet`。
給定 map 點 P，先確認 occupancy 為 free 且位於選定賽道走廊，再執行：

1. 在參考樣條中線找局部距離最小的投影 Q（閉合為週期曲線、開放為有限曲線）。候選滿足 `(Q−P)·曲線切線=0`，
   且距離平方的二階導數為正；程式解各樣條段的駐點，不只選最近 CSV 點。
2. 按 P 到 Q 的歐氏距離排序，先取最近候選。若另一候選的距離差≤半個像素，
   且兩個候選的中線弧長距離>1 m（閉合取最短環狀距離，開放取一般距離），就回 `ambiguous_projection`。
   ESP 解析度0.05 m，因此距離差門檻為0.025 m；這些是目前工程設定。
3. s 為起點到 Q 的中線弧長；令 t 為單位切線、n=(-t_y,t_x)，
   計算 `d=(P−Q)·n`。左側為正、右側為負。
4. 最近候選必須具有有效邊界、`d_right<d<d_left`、`J=1−kappa*d≥0.2`，
   並且 `|reference(s)+d*n−P|≤0.0001 m`。任一條件不符即拒絕，
   不自動改選較遠候選，也不先排除無效候選再判定歧義。

反向 `to_cartesian(s,d)` 先以同一公式計算 x/y，再用單點投影確認來源 s/d：
環狀 s 差及 d 差均須≤0.0001 m，且來源 J 也必須≥0.2。
即使該 x/y 可用另一分支表達，原來源不一致仍拒絕；診斷 x/y 可以保留。
目前 `ambiguous_projection` 同時用於單點近似等距歧義，以及反向轉換的來源
s/d 不一致；兩者不能只靠原因代碼區分。兩個候選存在本身不會觸發拒絕。
例如 AUT 來源 s=9.393393、d=−0.913537 產生的 x/y，單點查詢可接受
最近的 s=10.214952、d=−0.890108，但反向查詢拒絕原來源，因為回來的 s
差約0.822 m，超過0.0001 m。道路上的點仍存在；原截面的側向座標不能
直接作為目前「最近投影」介面的一致規劃參數。這是現有介面的選擇規則，
不是說該來源公式算不出 x/y；若未來明確保留局部投影分支，可另外定義
分支一致的轉換。但不能只靠有候選就放寬 J、占用或車身檢查。

急彎可能讓同一個 P 對同一條中線產生多個候選，並非有兩條中線。
工具會畫出候選供診斷；候選存在不代表都有效，也不代表一定歧義。
點選查詢依上述靜態幾何規則；連續定位的 FrenetTracker 另利用歷史與運動條件，
不應把畫面上的單點判定當成已完成實車定位驗收。
帶圖案例見[ESP 急彎的兩個候選投影](viewer.md#急彎的兩個候選投影)。

## ROS2 定位介面

目前提供 Python 適配器，沒有 ROS2 訂閱、TF buffer、publisher 或已部署的
Frenet 節點。部署端取得 TF 後呼叫下列入口；資料流是
`TF → 時間／框架／偏移檢查 → Frenet 投影與歷史追蹤 → 結果交給上層`。

```python
from f1tenth_benchmarks.research.core.geometry import FrenetTracker

tracker = FrenetTracker(geometry, source_reference='base_link',
                       offset=(0.0, 0.0, 0.0), offset_calibrated=False)
result = tracker.update_tf(transform, now=clock_seconds,
                           geometry_id=geometry.geometry_id)
```

geometry 已由 load_geometry 載入；transform 與 clock_seconds 由部署端提供。
零 offset 是範例，不能視為已量測或代表後軸。

| 資料 | 規則 |
|---|---|
| header.frame_id | 必須是 map |
| child_frame_id | 必須符合 source_reference，預設 base_link |
| transform.translation.x/y | map 下來源物理點的位置；目前使用平面 x/y |
| transform.rotation | 有限且非零的四元數，適配器正規化並取 yaw |
| header.stamp.sec／nanosec | 原始定位時間；不以接收時間替代，nanosec 在合法範圍 |
| now | 與來源 timestamp 同時基的目前秒數，用於時效檢查 |
| geometry_id | 必須等於當前載入幾何版本 |
| offset=(x,y,yaw) | 在來源車身框架定義的固定偏移，旋轉後套到 map；部署端須量測 |

不經 ROS2 的定位資料可使用
`tracker.update([x,y], yaw, timestamp, now=clock_seconds, geometry_id=geometry.geometry_id)`。
一般 update 可省略 now／版本作離線檢視，但 freshness_verified／version_verified
相應為 false；TF 入口要求兩者。offset_calibrated 明確標示偏移是否量測。

TrackingOptions 預設：最大定位資料年齡0.10 s、相鄰時間間隔0.25 s、速度上限
15 m/s、加速度上限20 m/s²、位置餘裕0.15 m、進度餘裕0.25 m、航向容差1.2 rad。
這些是可配置工程值，尚未由實車校準。只有唯一符合幾何與歷史條件的候選才接受。
拒絕時保留上次有效狀態，後續需明確 `tracker.reset()`；版本切換用
`tracker.replace_geometry(new_geometry)`。失效結果不可當成新定位使用。

## 保存資料與其他介面

幾何目錄以 geometry_id 識別，manifest 記錄來源、選項與檔案 SHA256；
reference_curve.json 保存樣條，中線 CSV 保存 x/y/s/方向／曲率，邊界 CSV 保存
逐截面的左右 x/y 與有號 d。waypoints_ros.csv 有 x,y 表頭，供既有 ROS loader
讀取；它只提供路徑位置，不代表新節點或控制接線已完成。
C 域與報告另存，schema_version=1；英文檔名 pose_contract.json 表示定位介面規格。

連續圈 planner 介面維持 `planner.plan(observation) → [steering, speed]`。
`core/continuous_sim.py`、`core/lap_tracking.py`、`core/logging.py` 轉匯出既有
研究實作，不複製算法。行車紀錄 samples.jsonl 的 physical_state 依序為
`[x,y,steering,signed_speed,yaw,yawrate,beta]`，simulation_time 為實際步進時間；
圈事件的跨線估計與實際物理樣本分開保存。介面不可混接不同幾何版本的進度。

## 本機圖形介面與 D 報告

viewer 使用標準庫 ThreadingHTTPServer，僅綁定127.0.0.1，沒有新增網路框架依賴。
GET /api/catalog 取得已載入版本，/api/map?map=INDEX 取得圖層，/api/image 取得地圖。
/api/query?map=INDEX&c=C&v=V 由 Python 將像素中心索引轉成 map 後呼叫 to_frenet；
/api/inverse?map=INDEX&s=S&d=D 另外檢查指定來源截面。前端只處理畫布視角，
不重新實作 Frenet 公式。非法參數回400，未知路徑回404，不公開任意檔案路徑。

回傳 frenet 是該 x/y 的單點投影；source_frenet 是反向查詢的原 s/d 有效性。
原來源無效時，另一個有效投影只作診斷顯示，不能當成原來源通過。
query_pixel 使用畫布格子邊緣座標；pixel 欄位是像素中心 c/v，兩者相差0.5像素。
切換目錄／版本清空查詢，結果帶 geometry_id，防止混讀版本。

D report 保存有限取樣與加密結果，numeric_checks_passed 不等於全域批准；
它表示目前數值轉換檢查通過，不表示所有地圖位置都適合單一 Frenet 表示。
目前 interpolation_allowed=false、continuous_verified=false；即使部分線段的
中點／四分點抽查通過，執行時邊界仍用 exact_grid_ray。

## 開放參考線的端點規則

manifest.closed=false、reference_curve.periodic=false 表示開放參考線。
reference／boundaries／section 僅接受[0,L]；HTTP 超界回400，
to_cartesian 超界回 valid=false、reason=outside_reference、xy=null。
開放投影的 s 差使用一般距離，不用環狀距離；FrenetTracker 不累加圈數，wrap_count=0。
曲線端點之外的點不以外插或繞圈假造座標。單點判定與 J／重建門檻沿用閉合圖規則。
舊 build_domain 為閉合 Stage C 產物，對開放圖明確拒絕；改用 section_ranges，
DomainAtlas.section 本身仍為逐點必要條件查詢。

/api/map 新增 closed、range_summary，/api/section 帶 closed 與 s_domain_m。
範圍產物讀取會核對 geometry_id、來源 YAML／影像與檔案雜湊；s=0 與方向
不能跨版本混用。CornerHall 的靜態幾何與開放追蹤規則已測，ROS2 部署仍另行驗收。

CornerHall 的 waypoints_ros.csv 保留有限路徑的兩個端點；CSV只提供x/y，
使用端須讀取 manifest.closed，選用開放路徑處理。原有假設閉合繞圈的 benchmark
規劃器不能僅靠讀取這份CSV就視為完成 CornerHall 控制接軌。


## 正式資料載入

檔案格式與候選位置見 [maps/frenet/README.md](../../maps/frenet/README.md)。

正式 ranges/manifest.json 使用 csv_location=source_geometry；load_ranges 必須
傳入 geometry_path，對該目錄的兩份CSV與幾何manifest核對hash，避免重複CSV。
舊本機bundle仍支援CSV在同目錄與普通JSON，格式選擇不影響轉換算法。
