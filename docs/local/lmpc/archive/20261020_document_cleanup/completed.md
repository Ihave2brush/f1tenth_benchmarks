# LMPC 已完成內容與目前狀態

## 10/20 最新：A5-COMPARE 四組控制變因

同一NO history與exact接手點，四組全新獨立run：C00 original/Reduced施加23步／6.988937m，C01 original/Full為6步／1.774285m，C10 NO/Reduced及C11 NO/Full皆6步／1.772930m；全部QP迭代上限停止、0圈、無碰撞／實際超限。已施加解皆通過原限制與共同KKT。共同6步NO跨目標17→0、rate反向19→5，但yaw／vy／ey預測RMSE略升；Full縮短cycle但沒有提升距離。487保護檔未變，production未改、無retry／投影／新PP。此批原模式history仍固定為NO，不是各模式完整初始化對比。見[今日進度](../../progress_20261020.md)及[四組完整報告](../../../../../Data/research/lmpc/experiments/A5_COMPARE_20261020_001/report.md)。下方舊實驗均作歷史，不混作四組數據。

## 10/20 最新：A5-NO-FULL 閉迴路未通過30步

沿用NO history／接手state與non_overshoot，僅以獨立Full physical backend接入；6命令、1.772930m、0.6s後QP7達100000次，未retry。前六輪原限制／KKT通過，QP7主要負lambda4.137e-5及dual未收斂；新QP可行性未另驗證。yaw一步RMSE1.09255rad/s，command快速反向、輪角速率飽和，但60tick無跨目標過衝，無碰撞／實際超限。成功cycle平均60.85ms、失敗451.06ms。包含一次純JSON故障的exact檢查點接續，首步未重求。仍無成功LMPC圈；production未變。見[今日進度](../../progress_20261020.md)與[完整報告](../../../../../Data/research/lmpc/experiments/A5_NO_FULL_20261020_001/report.md)。下方QP-D1為先前離線成果，不能當成整段閉迴路已驗收。

## 10/20 最新：QP-D1 離線對照

Full physical在相同OSQP0.6.3與原設定下，同時成功首輪／第二輪固定QP（1575／1050次），605條原限制最大違反5.20e-16／5.55e-16且KKT通過；與成功Reduced解／成本一致。原Reduced saved QP2失敗重現；Reduced all亦可成功，不能單獨歸因null-space或近零列。Full為優先離線候選，production未變、無新閉迴路、仍無成功LMPC圈。數值報告與獨立重現程式見 [10/20進度](../../progress_20261020.md)；以下10/19與10/15紀錄保留各自時點。

截至2026-10-19（Asia/Taipei）。**仍沒有成功LMPC整圈。non_overshoot致動器修正通過本次回歸，新初始化正常；A5-NO在第二次QP停止，S1僅關內部scaling後在第一次QP停止。** 原A5的22步、2.2s、6.6811m保留為該組歷史基準，不能視為持續學習改善。最新結果見 [10/19進度](progress_20261019.md)，A0～A8歷史見 [10/15進度](progress_20261015.md)。

## 10/19 最新驗收與結論修正

| 層次 | 本日完成／界線 |
|---|---|
| 致動器 | 可選non_overshoot，預設original；53項新／舊回歸通過；原A5 28,660物理子步重播誤差0；固定命令與PP不跨目標 |
| PP性能 | original／NO各完成兩圈；直線yaw／vy RMS降低，但最大lateral error增加，不能概稱全部改善 |
| NO初始化 | 原A5速度、PP、acceleration介面與激勵；三圈新資料通過驗收，2,843 WLS transitions；不混用原A2 |
| A5-NO閉迴路 | 1命令、0.1s、0.262588392m、0圈；第二次QP達100,000；沒有碰撞或實際狀態超限 |
| QP2診斷 | 離線重建完全重現保存status／iterations／residual；原限制可行且可得KKT解；主要違反lambda非負；非數學不可行 |
| scaling候選 | QP2 scaling0於3,325次成功，但S1首輪100,000次失敗，0命令；不能全域套用scaling0 |
| 數值表示 | 近零reduced rows產生約2.366e14 bound、低曲率Hessian；與失敗因果尚未隔離證實 |
| 可觀測性 | S1已保存執行時完整原始／solver QP、最後x/y、模型、terminal與設定；QP2仍須標為離線重建 |
| 原始實作研究 | 作者模擬有terminal/lane slack、完整QP直接送OSQP；BARC wrapper接受max iterations；無法確認論文實車當時部署細節 |

只有一個NO已執行transition，不能據此驗收整體ATV改善；S1沒有新transition，預測誤差無法評估。今日未調WLS權重、加Trust Region或修改production QP。原始報告的下一步保留原時點語意；下方10/15表格亦為歷史範圍。

## 已完成且可沿用

| 內容 | 完成範圍與界線 |
|---|---|
| core Frenet／圈事件 | 重用中線、曲率、正反轉換與連續進度；六維觀測與Continuous圈事件已在LMPC層接線 |
| 真實初始化與Lap Database | 三圈PP、前兩來源各有真實連續延伸；source IDs、命令／物理狀態／buffer／後繼／hash可追溯；A8使用其專用plant新收集資料 |
| Local Safe Set／Q-function | 每來源20點，共40欄D/S/J；同一λ、六維hard terminal、LP與原單位殘差核對 |
| 六維paper模型骨架 | 五係數WLS、Frenet Euler、固定ATV與上一解移位；骨架與合成例不等於多步預測通過 |
| 基本QP與10/14加速 | X索引、literal h分支、active-goal終端成本、尺度／零空間處理；四固定跨線窗口20次解一致且QP<100ms，僅適用原測試範圍 |
| 連續runner | 保留physical state、buffer、時間、previous_input與session/local s；只施加u0；失敗資料不加入成功來源 |
| A0～A4對照 | PP速度／激勵／探索上限、側向外推與固定QP收斂診斷完成；仍無整圈 |
| M1／T1離線診斷 | 致動器精確重播、預測平均角一步改善、nominal/oracle差異與ATV放大已量化；多步及可部署修正尚未通過 |
| A5／A6選點變體 | 含輸入L1／h15、command或actual角選點；回歸仍command，分別22／20步；非PDF原Q/h10方法 |
| A7／A8獨立對照 | 七維近似致動器模型及無FIFO六維plant已建立並短程試跑；未證實能解決預測／控制問題 |

程式主體位於`f1tenth_benchmarks/research/lmpc/`，設定位於`params/research/`。A7增加維度、A8改plant均須明列，不能把所有變體合稱原論文完整復現。

## 10/15 歷史驗收結論

| 層次 | 目前結論 |
|---|---|
| 原始紀錄一致性 | A2／A3／A5／A6／A5歸零版的既有獨立audit通過，plant重播／已執行等式／歷史NPZ均有核對；不等於完整成功圈 |
| 本次檔案檢查 | 整理前222個今日產物；99份JSON、59份JSONL可解析，25個NPZ ZIP CRC通過；11個session控制步數及yaw RMSE與逐筆紀錄吻合 |
| 獨立預測 | 平均角改善一步，但固定ATV多步仍有失準與發散；oracle未來真實查詢不能作線上方案 |
| 實際短程控制 | A0/A1/A2/A3/A4/A5/A6/歸零版/A7/A8分別1/1/2/4/7/22/20/19/7/6步；均0成功圈 |
| QP | A1有LP不可行診斷；A6回報primal infeasible；A4/A5/A7/A8迭代超限。不同失敗原因分開保留，未收斂不冒稱無解 |
| 即時性 | 今日A4/A5/A6/A7/A8完整流程最大約701/662/190/697/653ms，未通過；不能用10/14四窗口QP benchmark取代 |
| 測試 | 保存報告各範圍有重疊，不相加成總數；本次文件整理未重新執行單元測試或模擬 |
| 持續學習／改善 | 成功圈自動入歷史、最近兩來源與探索上限更新、每10圈與賽車線比較均未驗收 |

今日關鍵問題有三層：致動器／命令與實際轉角過渡的表示、nominal選點與實際/QP應用點的局部外推及ATV放大、固定QP的不可行或數值收斂。rank滿、80筆正權重或小fit RMSE均不能單獨證明模型可靠。

## 歷史成果保留

- B1舊兩圈PP約196.93949／196.72221s，39380物理子步重播通過；B2 observed-only與128點periodic為早期介面證據。現行原plant實驗採三圈真實連續來源，模板不是新量測。
- B3舊hybrid最長171步、17.1s、s≈23.88675m，非成功整圈，亦非今日paper變體的性能證明。
- 10/14原模型第一步接手與QP精度／加速證據見 [10/14進度](progress_20261014.md)。今日新增模型／限制的失敗不追改舊驗收結果，舊驗收也不自動涵蓋今日變體。

## 接續工作（10/19更新）

1. 固定保存的首輪與QP2，優先隔離近零列正規化、多層縮放與等式消去的等價表示問題；不先新增閉迴路。
2. 硬terminal、低曲率成本與資料幾何另行評估；若改slack／成本，明列原問題已變更，不當作純solver修正。
3. 候選修正須通過多個固定QP與全部原始限制，再評估短程／整圈與完整控制時間；目前尚未批准可通用的設定。

## 10/15 接續工作（保留歷史）

1. 固定今天資料，隔離驗證離散致動器、因果名義軌跡重建與局部模型多步穩定性；不把oracle當可部署控制。
2. 固定失敗QP分析尺度、等式消去與活躍限制，維持原約束／gate；診斷性取消限制的解不直接施加。
3. 預測、短程、完整圈與完整100ms流程分別通過後，再做成功歷史更新、連續學習及性能比較。

core全域planning/domain/continuous認證仍未完成。另有 [環境接軌計畫](../../../../project/integration_plan.md)，僅屬後續規劃，不代表雙車／超車／ROS2／實車功能已完成。方法對照見 [論文與變體](../../paper_method.md)，來源盤點見 [檢查紀錄](../../checks/document_review_20261015.json)。
