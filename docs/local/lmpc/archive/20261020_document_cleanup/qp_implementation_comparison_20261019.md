# QP／Terminal Safe Set 原始實作對照

研究日期：2026-10-19。唯讀研究，未執行作者碼、重求 QP、調整 production 或新增閉迴路。今日實驗摘要見 [progress_20261019.md](progress_20261019.md)。

## 來源與可確認界線

- 使用者論文：Rosolia／Borrelli，Learning How to Autonomously Race a Car: A Predictive Control Approach；本機 PDF `/home/kimi/Zotero/storage/VSQGAU4J/2020_Learning How to Autonomously Race a Car.pdf`。
- RacingLMPC 固定 commit `b49c95e7795ef6f343efacedb6b90a69c7855b32`，2020-10-01；下列模擬碼均以此版本連結。
- 論文註腳所指 BARC devel-ugo，本次可讀 commit `7a481c7044fb85aacd64f2b36df9e29463ea5912`。
- 本地來源根目錄 `/data/f1tenth/sim/f1tenth_benchmarks/`；本地行號為此次核對時工作區。

公開分支不等於已證實的論文實車部署版本。實車當時 commit、套件版本、啟動參數、solver 分支與上層失敗處理仍無法由本次來源確認。既有舊來源快照留在 `sources/r0_20261012/`，本次沒有覆寫它。

## 論文數學要求

式 (7)、(8c)：lambda 非負、sum(lambda)=1、xN=D lambda；式 (8d)：ATV 動態等式；式 (8e)：X/U。式 (8a) 是到達終點前的 stage cost 加歷史線性 terminal cost。Section V 提到 OSQP 與小 input-rate cost，未給完整 solver settings，也未在式 (8) 寫入 terminal／lane slack。

## RacingLMPC 的 QP 建立

主要來源 `src/fnc/controller/PredictiveControllers.py`：

| 函式／行號 | 實際做法 | 固定來源 |
|---|---|---|
| MPC.buildIneqConstr，166–198 | block state/input limits；可加非負 state slack，輸入仍硬限制 | [code](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L166-L198) |
| MPC.buildEqConstr，200–226 | 完整狀態下的 x0 與 x[k+1]-A[k]x[k]-B[k]u[k]=C[k]；slack 動態係數為 0 | [code](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L200-L226) |
| MPC.buildCost，228–257 | Q/Qf、R、差分 input-rate；前次輸入形成 q；state slack 二次與線性懲罰；H 乘 2 對應 1/2 z'H z | [code](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L228-L257) |
| LMPC.addSafeSetIneqConstr，340–343 | -I lambda <= 0；terminal slack 無正負限制 | [code](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L340-L343) |
| LMPC.addSafeSetEqConstr，345–357 | xN-D lambda+terminal_slack=0；sum(lambda)=1 | [code](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L345-L357) |
| LMPC.addSafeSetCost，359–362 | lambda Hessian block 為 0，q 為歷史 cost-to-go；terminal slack Hessian=2 QterminalSlack | [code](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L359-L362) |

決策變數順序：states、inputs、lane slack、lambda、terminal slack。沒有 lambda 二次 regularization；terminal slack 不會放寬 lambda 非負或 sum=1。

`src/initControllerParameters.py::initLMPCParams()` 第 28–58 行的實際配置：lane ±map.halfWidth，input delta±0.5、a±10；最快 4 條歷史，共 48 terminal 點；QterminalSlack=500 I；lane 實際懲罰 5 ε²+25 ε；Q/R=0，dR=[5,50]，slacks=True。[固定來源](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/initControllerParameters.py#L28-L58)。註解的 Qslack 順序與實際 buildCost 解讀不同，以實際矩陣為準。

Safe Set：`addTerminalComponents()` 395–407 行排序最快圈；`selectPoints()` 478–499 行以六維未加權 L1 找最近點，再取相鄰時間窗口；`feasibleStateInput()` 382–384 行用 successor 凸組合更新 candidate。[選取來源](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L386-L407)、[窗口來源](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L478-L499)。

公開 main N=14、vt=0.8，並將 PID 軌跡重複四次加入 LMPC 初始 model/history，不能稱為論文兩圈 1.2 m/s 實車初始化的精確重現。[main，43–50／101–110 行](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/main.py#L101-L110)。

## OSQP 與失敗處理

`MPC.osqp_solve_qp()` 259–283 行：把 inequality F、equality G 堆疊，l=[-inf,b_eq]、u=[b_ineq,b_eq]；每次新建 OSQP，只顯式指定 verbose=False、polish=True。scaling、rho、adaptive rho、eps_abs/rel、max_iter 均使用當時安裝版本預設；repo 無固定依賴版本，不能推定當年數值。[固定來源](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py#L259-L283)。

wrapper 支援 initvals，但 `MPC.solve()` 呼叫未傳；因此沒有明確跨控制時刻的 OSQP x/y warm start。前次軌跡平移用於下一輪線性化，兩者不同。未見 retry／solver fallback。

模擬 wrapper 只把 solved 設為 feasible=1，但仍保存 res.x；`Simulator.sim()` 33–40 行不檢查 flag 就取 uPred[0]。[模擬迴圈](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/simulator/SysModel.py#L33-L40)。不能從「程式繼續跑」推斷所有 QP 都已收斂。

## BARC 公開分支：不能與模擬碼混稱

`workspace/src/barc/src/ControllersObject/LMPC.py`：

- `_LMPC_BuildMatCost()` 398–453：lambda 線性歷史成本，terminal slack／lane slack 有懲罰。[cost](https://github.com/MPC-Berkeley/barc/blob/7a481c7044fb85aacd64f2b36df9e29463ea5912/workspace/src/barc/src/ControllersObject/LMPC.py#L398-L453)。
- `_LMPC_TermConstr()` 626–663：同樣有 terminal slack，sum(lambda)=1。[terminal](https://github.com/MPC-Berkeley/barc/blob/7a481c7044fb85aacd64f2b36df9e29463ea5912/workspace/src/barc/src/ControllersObject/LMPC.py#L626-L663)。
- `osqp_solve_qp()` 374–396：完整矩陣直接堆疊；polish=True；沒有顯式 scaling／rho／容差。甚至將 SOLVED_INACCURATE 與 MAX_ITER_REACHED 設為 feasible=1。[solver](https://github.com/MPC-Berkeley/barc/blob/7a481c7044fb85aacd64f2b36df9e29463ea5912/workspace/src/barc/src/ControllersObject/LMPC.py#L374-L396)。

這是公開來源的狀態判斷，不是原始限制已驗收的證據；無法確認論文實車當時的啟動路徑及上層處置，不應照搬接受 max iterations 的策略。

## A5-NO 額外處理與定位

本地 `f1tenth_benchmarks/research/lmpc/controller.py::solve_qp()`：

| 行號 | A5-NO 處理 | 與作者／論文關係 |
|---|---|---|
| 22–38 | N12、六維狀態偏移／輸入偏移／40 lambda，共 142 變數；僅 rate quadratic、terminal linear cost 減共同常數 | lambda 成本減常數在 sum=1 下不改解；rate 權重是專案選擇 |
| 49–67 | x0、12 階動態、六維硬 terminal；SVD 等式轉換 floor=.05；0<=lambda<=1 | hard terminal 對應式(8c)；SVD 與冗餘 lambda 上界是額外處理 |
| 70–125 | heading/lateral、32-sided speed polygon、local vx、input 與 local steering、finish branch | 平台限制／探索限制與分支表示，不是作者模擬原配置 |
| 132–140 | state scale=[1,1,1,.1,1,.1]、input=[.1,1]、lambda=1/40；列 equilibration | 專案外部數值處理 |
| 142–169 | HiGHS LP reference、null-space 消去全部硬等式 | 作者所查完整 QP 路徑沒有此步 |
| 170–174 | reduced inequality 再除以 max(row norm,1e-12) | 近零列／巨大 bound 的來源 |
| 175–209 | speed facet constraint generation、同時刻 workspace reuse | 額外精確限制生成，成功後仍檢查完整問題 |
| 192–199 | eps_abs=eps_rel=1e-7、max_iter=100000、polish=True、refine_iter=10、adaptive rho=True、interval50 | 比作者顯式設定更詳細；0.6.3 預設 scaling10、rho.1；S1 僅 override scaling0 |
| 211–247 | 非 solved 立即失敗；成功後物理／原限制驗收；成功 lambda 有 roundoff clamp/normalize | 不接受 max iterations；後處理不是失敗 QP 的修復 |

六維分支沒有啟用九維 prediction_trust_box；不要因 YAML 有欄位就稱 A5-NO 有 Trust Region。完整 finish indicator 分支見 `solve_finish_qp()` 251–336，這不是作者模擬中的相同 finish 處理。

`safe_set.py::select_local_safe_set()` 78–108：最近兩成功來源各 20 點，依 s 距離選點，與作者最快四圈、六維 L1 加時間窗口不同。Safe Set 離線 `q_function()` 的 rank 處理不是控制 QP 的等式消去，不能混用結論。

## 與既有未收斂現象的關係

已證實：QP2 原始限制可行；最後主違反是 lambda 非負，不是動態／terminal 等式；scaling0 或固定 rho 均曾解出同原問題。S1 卻在首輪 scaling0 失敗，不能宣稱有通用 solver 參數修正。

已證實的表示問題：近零 reduced rows 產生最大約 2.366e14 finite bound；reduced Hessian 正曲率比約 8.39e7；仍有 33 個零曲率方向。terminal centered matrix full rank，不支持「精確 rank 缺失所以無解」。

尚未證實：巨大 bound 是否透過內部 scaling／rho 耦合造成 lambda 收斂敏感；hard terminal 是否是主要難度來源；更換作者 slack／成本是否改善閉迴路。加 slack 或改成本會改原問題，須獨立標示。

優先驗證：固定首輪與 QP2 的等價矩陣表示及近零列；其次 terminal 硬匹配與低曲率幾何；最後核對收斂／接受標準與套件版本。此處為研究建議，沒有執行新實驗。詳細數值與原始證據連結見[今日進度](progress_20261019.md)。
