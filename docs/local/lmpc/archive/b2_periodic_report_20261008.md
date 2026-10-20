# 改進 B2：既有兩圈的週期終端歷史

日期：2026-10-08（Asia/Taipei）。**改進後 B2 已驗收：第三圈可以使用前兩圈作歷史資料，跨終點窗口使用經檢查的圈頭重用；未增加初始化完整圈，尚未開始 LMPC 行車。**

2026-10-12 複核註記：本報告保留 10-08 的連續接手方案及數值驗收。最新方法為「兩圈歷史資料＋週期延伸，LMPC 新實驗重新從起點靜止開始」，不使用 resume_snapshot 恢復 B1 終點；舊 cycle=2 候選也須依新實驗座標重算。原始點數、K=20、三個 NPZ、s 與 reset 等討論已整理在 [B2 重新檢視紀錄](b2_review_20261012.md)。本次未修改或重新生成原數值證據。

此方案依使用者指定取代原 B2「只能使用實測連續後綴」的限制。原始實測資料與初版驗收報告保留，不將週期模板冒充新量測。

## 如何接續

第一圈已有實測第二圈作終點延伸，維持原樣。第二圈另取其圈頭下一個控制樣本開始，共 128 個已觀測樣本：source rows **1971～2098**。它們在 B1 結束 row 3937 前早已取得。

每個模板狀態的 s 加上同一中線長度 L=236.907497773 m；其他狀態、物理位置、歷史控制、source ID 及原 timestamp 保留。模板的虛擬步序只用來對齊終端成本，不能當成新的實際量測時間。

第二圈最後狀態的成本為 0，模板成本為 -1、-2……-128。每個後繼的成本比前一個少 1；這是原 B2 已明列的有號延伸勢函數，不代表負的實際行駛時間。

資料分成三種：

| transition_kind | 意義 | 可作模型辨識資料 |
|---|---|---|
| observed | 原兩圈真正連續量測的轉移 | 是；仍須符合辨識選點條件 |
| approximate_periodic_seam | 第二圈最後樣本 → 重用圈頭首個模板 | 否 |
| historical_periodic_replay | 模板內重用過往後繼 | 否；使用原量測轉移避免重複算入 |

原始 `Lap.states`／commands／sample_ids／times 等陣列完全不改。週期部分另存 `PeriodicTail`；Local Safe Set 的 terminal pool 才加入模板與分類。

## 為什麼需要檢查接縫

Continuous 保證此次行車跨終點沒有 reset，以及使用同一連續 core 中線，不保證每圈完整動態狀態週期相等。控制取樣每 0.1 s，1.2 m/s 約每步前進 12 cm，終點可能在兩個樣本之間；PP 也允許小幅側向／航向誤差。

第二圈起點樣本與終點樣本的跨線餘量不同。直接重用圈頭，會產生有限接縫差異。此處保留該差異並量測，而不是插值修改所有歷史狀態或宣稱精確動力學後繼。

## 原模型接縫驗收

使用真正的 B1 結束物理狀態及 steering buffer，**不 reset**，套用歷史 row 1971 的控制與逐子步 PID target，讓原版 DynamicsSimulator 推進 10 個 0.01 s 子步。這是離線診斷預測，不是新增第三圈量測。每個子步 pre/post 的完整矩形車身檢查通過；128 個模板的 Frenet／車身候選檢查通過。

| 指標 | 實測診斷差異 | 工程門檻 |
|---|---:|---:|
| 原模型一步預測至模板首點的 Cartesian 位置差 | 0.026680123 m | 0.05 m |
| s 差 | 0.026651963 m | 0.05 m |
| e_y 差 | 0.001225487 m | 0.02 m |
| 航向差 | 0.008556449 rad，約 0.49° | 0.03 rad |
| e_psi 差 | 0.008561752 rad | 0.03 rad |
| vx 差 | 1.74434e-6 m/s | 0.05 m/s |
| vy 差 | 0.000189211 m/s | 0.03 m/s |
| yaw rate 差 | 0.000155350 rad/s | 0.30 rad/s |
| 實際轉向角差 | 0 rad | 0.02 rad |
| 接縫前，實際 endpoint 與原圈頭 buffer 差 | 0.011901158 rad | 0.02 rad |
| 一步後，預測與模板 buffer 差 | 0 rad | 1e-12 rad |

門檻在 `params/research/lmpc_periodic.yaml` 明列，於第一次原模型接縫診斷前設定，不修改 B1 已冻结設定或論文參數。r 的 0.30 rad/s 門檻對應 0.1 s 内最多 0.03 rad 的航向貢獻；實際殘差遠小於此門檻。這些是研究工程容許值，不是論文數值或安全證明。

`measured_seam=false`、`exact_dynamics_successor=false`；門檻通過不使其變成精確或實測後繼。

## Safe Set／Q-function 驗收

- 最近兩個來源圈各選 K=20，終端矩陣仍為 6×40。
- 423 個窗口包含均勻 s 掃描、精確終點、最後實測樣本及模板首 20 點；全部與獨立排序 oracle 一致。
- 查詢範圍 **0～L+9.6 m**；9.6 m 取 N×dt×vehicle.max_speed=12×0.1×8。這是目前參數下的終端候選查詢支援範圍。
- 來源 sample_id 與 successor source_id 全部 ≤3937，沒有未來資料或新觀測圈。
- 四個終点附近／延伸窗口的 Q-function、已知成功狀態重建、凸組合最小化及同 λ 後繼推進通過；最大原始六維等式殘差約 **1.88e-12**。
- 接縫及所有模板的成本遞減 1，圈座標加 2L 後來源與成本一致，對應第三圈的全域進度。
- 模板為有限長度；最後模板點沒有後繼時仍拒絕，不自動無限複製。
- NPZ roundtrip、分類、來源／政策／程式 hashes 核對通過。超門檻 audit 即使 `passed` 被設 true 仍拒絕；未來來源、修改狀態／成本與檔案損毀皆有測試。
- B1 原物理重播 39380 子步、78760 次車身檢查、Frenet／事件重建重新通過。
- 合併 B0／B1／B2／Periodic／Continuous 共 **43 項 unittest 通過，2.794 s**。

本次亦修正 Q LP 的數值處理：以固定資料原點建立 affine SVD basis，避免候選目標改變中心後造成近相依等式的數值問題。奇異值门檻 1e-8、求解前 affine-span 檢查與求解後原始六維 1e-7 殘差門檻明列；沒有只憑求解器 status 接受結果。

![週期成本与終點窗口](../../../../Data/research/lmpc/b2_esp_periodic_20261008_002/database_overview.png)

## 保存與 B3 接線界線

最終結果在 `Data/research/lmpc/b2_esp_periodic_20261008_002/`：

- `database.json`、兩個 observed lap NPZ：原始成功圈與來源。
- `lap_002_periodic.npz`：另存週期模板與原 source times／IDs。
- `acceptance.json`：接縫殘差、工程門檻、窗口／Q／分類／來源驗收。
- `resume_snapshot.json`：真正 B1 最後樣本的物理狀態、buffer、時間、Frenet 與圈數，供 B3 不 reset 恢復。
- `database_overview.png`：虛線與空心方框明列模板，紅叉標記近似接縫。

`third_lap_history_ready=true`，`full_lap_terminal_support=true` 的範圍是**包含近似接縫的 s 鄰近點與後繼資料可用性**，不代表 MPC 可行或已學會。B3 還要恢復 core lap tracker／時鐘，實作辨識、Frenet 線性化、OSQP 與約束，並驗證接縫的模型／QP 誤差、完整一圈與控制時間。模型辨識只用原始實測轉移。

`b3_controller_ready=false`、`learning_started=false`；core 的 planning_allowed／continuous_verified 維持 false。本次没有放寬 core 權限或執行 LMPC。週期近似不提供論文理論中精確後繼的遞迴可行性保證，實際可行性在 B3 驗證。

從 repository 根目錄，在現有模擬 Python 環境執行；輸出目錄須不存在：

```bash
python -B -m f1tenth_benchmarks.research.lmpc.build_periodic Data/research/lmpc/b1_esp_centerline_20261007_001 --output-dir NEW_PERIODIC_DIR
python -B -m f1tenth_benchmarks.research.lmpc.plot_database NEW_PERIODIC_DIR
python -B -m unittest discover -s tests/research -p test_lmpc_periodic.py -v
```

計畫、報告及結果依原要求保留在本機忽略目錄。未 commit／push。前版原始 observed-only 證據保留作比較。
