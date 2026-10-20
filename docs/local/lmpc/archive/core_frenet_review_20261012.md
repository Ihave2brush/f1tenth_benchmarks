# R0：core Frenet 功能覆蓋檢查

日期：2026-10-12（Asia/Taipei）。方式：檢查目前程式、介面文件與既有驗收資料；本次沒有執行新測試、模擬或修改 core。主計畫：[Section III、IV 完整復現計畫](lmpc_plan.md)。

## 結論

core 已提供本研究需要的基本 Frenet 幾何與實際車輛位姿追蹤。不需要重新建立另一套 Frenet 轉換。但「所有功能都在 core」目前不成立：六維觀測組合、Continuous 圈事件接軌、論文運動學與線性化、候選車身驗證仍在 LMPC 層。這些位置差異不等同數學能力缺失，也不能把現有接線當作所有 Benchmarks 已共用。

## 功能逐項對照

| 需要的功能 | 實際入口 | 覆蓋與限制 |
|---|---|---|
| 同一參考中線、弧長 L、方向、起點 | core/geometry/extraction.py 的 ReferenceCurve；frenet.py 的 TrackGeometry | 已有；不可混用舊折線／raceline 的 s |
| 參考位置、切向、航向、曲率 κ(s) | TrackGeometry.reference(s) | 已有；閉合曲線查詢可接受跨圈 s，以週期參考查詢 |
| 實際位姿 X/Y/yaw → s/d/eψ | TrackGeometry.to_frenet(xy,yaw) | 已有；靜態最近投影、道路／占據／歧義／Jacobian 檢查，可能拒絕 |
| s/d/eψ → Cartesian 位姿 | TrackGeometry.to_cartesian(s,d,e_psi) | 已有；包含反查一致性，不能假定任何 Frenet 候選都有效 |
| 車輛連續定位與跨線進度 | FrenetTracker.update / update_tf | 已有；s_wrapped、s_unwrapped、d、eψ、時間／版本／來源點檢查、跨線与倒退；wrap_count 不等於成功完整圈數 |
| 左右道路範圍與局部有效性 | TrackGeometry.boundaries、domain、section_ranges | 已有精確格子射線及逐點／採樣診斷；插值与全道路連續域尚未認證 |
| 六維實際 LMPC 狀態 | lmpc/state.py 的 paper_state | 已接入；core 追蹤給出 s/d/eψ，vx/vy/r 由實際七維物理狀態取得，組合功能目前在 LMPC |
| core Frenet 與 Continuous 成功圈事件一致 | lmpc/lap_adapter.py 的 CoreLapTracker | 已接入 ESP LMPC／PP 收集；調整 L、實體終點與 origin shift，核對雙方 s 進度；適配類不在 core |
| 式 (12) Euler 預測 | lmpc/local_model.py 的 kinematic_step | 程式已有，使用 core κ；這是模型預測，不是位姿投影，尚非 core 共用動態 API |
| 式 (16)/(17) 位置線性化 | lmpc/paper_model.py 的 linearize | 程式已有中央差分與 affine offset；沒有在 core 提供現成 Euler/Jacobian，不能稱數學復現已驗收 |
| 預測軌跡 Cartesian 車身檢查 | lmpc/validation.py | 目前在 LMPC 層，core 提供逆轉換與 raster；geometry 單點 valid 不代表車身有效 |
| 所有 Benchmarks 自動使用 core Frenet | 各 runner 的接線 | 尚未全面整合；原 Continuous 預設仍為折線 tracker，LMPC runner 才明確換成 CoreLapTracker |

相對路徑均以 repository `/data/f1tenth/sim/f1tenth_benchmarks` 為根，core 全名為 `f1tenth_benchmarks/research/core`。

## 語意與介面注意事項

1. 靜態投影與連續 tracker 有不同歧義處理規則；狀態觀測用 tracker，候選反查可能更嚴格拒絕，不能任意互換。
2. `reference(s)` 可週期查幾何；這不表示歷史資料 s 要 modulo 歸零。Safe Set 距離比較在已對齊的連續 s 上做，不能直接用 geometry.distance_s 的環狀最短距離取代。
3. CoreLapTracker 會保存 native_s_unwrapped 與 origin_shift，對齊 Continuous 的研究起點；新實驗與來源圈平移由資料／生命週期契約處理。
4. FrenetTracker 不提供 vx/vy/r。vx=v*cos(beta)、vy=v*sin(beta)、r=yawrate 取自 plant；座標轉換不能替代速度動態。
5. core 沒有現成 κ 對 s 導數 API；目前位置模型中央差分會透過 reference(s±ε)求導。這是需驗收數值誤差的模型實作選擇，不是缺少 κ(s) 本體。
6. 幾何與 tracker 的工程 gate（如 J>=0.2、航向／時間／進度 gate）需列為環境差異。幾何 valid、domain_verified、planning_allowed 是不同概念。
7. 目前 load_geometry 需要 inspect=True，且全域 planning/domain/continuous 旗標仍未認證。現有研究使用獨立候選與原模型預演檢查，不改寫這些旗標；有限採樣通過不等於連續安全證明。

## 現有證據與限制

既有 tests/research/test_frenet.py 包含獨立圓環正反轉換、接縫、Jacobian、歧義与 hint；test_frenet_tracking.py 包含追蹤、時間／版本、偏移与拒絕等契約。這是測試覆蓋的程式檢查，不宣稱本次重新執行通過。

新 ESP 初始化 `Data/research/lmpc/b1_esp_observed_20261012_001/session.json` 已有 passed 證據；資料庫 acceptance 已有窗口与後繼證據。這些支持目前 ESP 接線，不證明所有地圖、所有 runner 或論文多步預測均通過。

## 下一步

保持現有 core 幾何與追蹤，先固定六維觀測／控制／時間／s 原點的資料契約，再審核論文 Euler 與梯度。是否把通用六維觀測或圈適配提升為 core API，列為架構討論；本次不搬檔、不重写 core、不將 LMPC 專用回歸／QP 放進幾何層。
