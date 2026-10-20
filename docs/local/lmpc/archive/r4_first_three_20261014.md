# R4：QP 精度、目標／約束、連續接手實作紀錄

日期：2026-10-14（Asia/Taipei）。依使用者「前三個請開始」完成實作與分項驗證。尚未完成 LMPC 整圈，未開始新成功圈的學習更新。

## 1. QP 精度

同一個初始 QP 的獨立 LP 可行性檢查成功，殘差約 1.38e-14；原 OSQP 調整 rho/adaptive/scaled termination 的 20 組結果仍不能達到 solved，並非據此可稱數學不可行。

在 `controller.solve_qp` 加入等價的變數／列尺度處理，以及 hard equality 的 numerical null-space 消去：令原變數 v=reference+Z*y，Z 是等式矩陣零空間，保留原始 objective 與不等式。LP 只提供 reference；未知 LP status 改用等式 particular solution，不當作 infeasible。數值 rank 使用機器精度尺度，未引入 terminal slack、ridge 或更寬的 gate；解還原後檢查原單位限制与六維 hard terminal。

實際接手第一解：OSQP solved、1675 iterations，constraint residual 約 1.33e-7、terminal residual 約 2.87e-6，通過原 1e-5 gate。OSQP 約 .0325 s，第一個完整控制決策約 .0850 s。這是單一控制步的證據，不代表所有後續步都能解或保持100ms。

## 2. 論文目標與約束對齊

- 狀態 X 索引改為 x0..xN-1；xN 另由同一 lambda 的 hard terminal 決定。初始量測也必須滿足 X。合成測試明確驗證 xN 不被多加一份 X。
- U 的 steering command 上限採 simulator .4189 rad，加速度 command 上限9.51；原 plant 的高速飽和、轉向 rate、buffer 不變。
- 去除舊 QP 的獨立 vy±.5、r±5 與 s 前進 box；速度cap8採32邊內接多邊形，最保守徑向差約 .4815%。vx>=0、heading±.5、ey±.25 是明列的 ESP 實驗 X，不冒稱論文／simulator公布的全部數值。
- `solve_finish_qp` 實際計入 h=1(s<L)。由 affine interval enclosure 能證明固定的 stage 不分支；其餘依 before/after finish 分成凸 QP，比較包含 h、terminal cost 與 input-rate 的 objective。不能再宣稱跨線全域仍只解一個 ordinary QP。
- 當前 finish 的歷史點 cost 在 finish 之後取0，與已到目標後的 h=0 一致；資料庫原有 signed continuation potential 保留，沒有覆寫。這是明列的 active-goal 延伸解讀，不是作者公開碼 offset 策略的逐行複製。
- 浮點邊界 within 原 equality gate 可按分支表示為 L 或 nextafter(L,-inf)，再核對原動態 residual；超過 gate 拒絕。分支 bound／QP optimality 採現有數值容差，不宣稱本紀錄建立了所有資料／狀態的形式等價證明。
- 終端 successor 的名義控制改取 u_(i+1)，不是原 D_i 的 u_i；缺少此 outgoing input 明確拒絕。
- 精確 plant shadow 僅驗證真正將施加的 u0（10個物理子步），符合 receding-horizon 的執行方式；不把假設未來12個控制全部開迴路執行的結果作額外 veto。保留整段候選的幾何／footprint 檢查、真實一步限制与碰撞檢查；core全域permission不變。

四個真實跨線窗口 raw IDs 3916/3922/3928/3934：4/4 solved且通過原殘差門檻；stage costs為9/7/4/2；QP分支數13/11/7/1。總分支時間約 .493/.646/.184/.016 s，前三個不達100ms；這個即時性限制尚未解決。輸出：`Data/research/lmpc/r4_literal_finish_qp_20261014_002/validation.json`。

## 3. 連續 PP→LMPC 正式 runner

paper mode 的 `run_lmpc` 現在在同一個 sim 收集三圈PP1.2m/s，獨立驗收後封存 `initialization/` 與 `initial_database/`。資料庫只有前兩個來源，分別用圈2／圈3的真實延伸。第三圈完成後不 reset，直接建立LMPC首次 flying歷史 seed，previous_input取最後PP command，第一nominal點取實際觀測。

原始 `paper_state.s` 始終保存 session 進度；額外 `lmpc_state.s=session_s-3L` 供控制使用。時間、七維physical state、buffer、sample ID與圈事件連續。`--max-steps` 在paper mode只計LMPC階段，不截掉三圈初始化。首次成功整圈才保存 experiment_lap_001.npz；failed probe不標成功、不加入歷史。

實際結果 `Data/research/lmpc/r4_continuous_handoff_20261014_003/`：

- PP跨線完成 elapsed196.8/393.6/590.3s；handoff物理time590.4s、reset_count=1、actual speed1.2m/s、local s=.0679168m。
- 第一次 QP solved，u0約 [delta=.0582362rad,a=2.7079237m/s²]；一步shadow通過，實際施加完整 .1s。到590.5s scalar speed約1.4707924m/s。
- 第一個 LMPC interval 的pre_state與最後PP post_state完全一致。獨立 original-dynamics replay 共59050 physics ticks，state tolerance1e-12通過，buffer亦一致。
- 第二次控制尚未呼叫QP，先遇到 `ATV model construction: insufficient weighted paper samples`；停止並保存failure.json，session passed=false。

## 4. 目前單一控制阻礙

第一次模型預測的 vy/r 約 .540829/5.107140；真實一步結果約 .025217/.162085，兩者差異很大。IV-C移位因此把前者當下一個nominal第一點。初始化歷史r範圍約[-1.0237,1.2742]；該nominal到全部歷史的最小 squared Q distance約14.8157，大於kernel h10，正權重點數0。

這次已經讓 QP 與第一步真實控制發生，證明新阻礙不是「係數不完整所以不准開始」，也不是第一QP精度。後續要处理局部模型／名義軌跡的資料支持與真實側向控制效果；不在此次自行改P/Q/h、加ridge、追加激勵或用fallback控制冒充LMPC。

## 5. 驗證與使用

LMPC測試共62項通過，涵蓋X索引、hard terminal、含h的跨線objective、符號延伸不作負剩餘時間、unknown LP初始化status、first-action shadow與既有回歸／資料契約。

正式paper入口：

```bash
python -B -m f1tenth_benchmarks.research.lmpc.run_lmpc \
  --config params/research/lmpc_paper.yaml \
  --output-dir Data/research/lmpc/<new_run> --max-steps 3
```

需要完整單圈時移除max-steps；本紀錄不保證目前模型能完成。舊 hybrid模式保持其獨立靜止runner語意。首次QP離線資料 `r4_qp_setup_20261014_003`、實際接手`r4_continuous_handoff_20261014_003`與跨線`r4_literal_finish_qp_20261014_002`分開判定。
