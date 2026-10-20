# 可切換轉向致動器：original / non_overshoot

2026-10-19。預設與歷史設定均保留 original，未修改全域 simulator_params.yaml。

建立模擬器時設定模式，例如既有 Continuous 介面：

```python
sim = F1TenthSim_Continuous(
    'esp', extra_params={'steering_actuator_mode': 'non_overshoot'})
```

直接使用 DynamicsSimulator 或 TracedDynamics 時，在傳入的 Namespace 設定相同欄位。未提供欄位等同 original。模式於初始化選定，不在同一份 history 中途切換；metadata 必須保存模式。歷史資料未標模式時採 original。

## 更新順序

1. 從兩格 FIFO 取出延遲命令，再放入本次 raw command。未填滿時取0，仍延遲兩個物理步。
2. 僅 non_overshoot 將取出的有效目標夾在 s_min/s_max。FIFO 保留 raw command；超界請求與合法有效目標在 trace 分別保存。
3. 呼叫未修改的 pid()，包含原 1e-4rad deadband、縱向控制與最大轉向速率。
4. 僅 non_overshoot 依原 steering_constraint 限速，再限制單步朝目標移動量不大於剩餘角度。
5. 將該速率交給未修改的 vehicle_dynamics_st/ks，再 Euler 積分。低速 yaw 導數使用同一有效轉向速率。
6. 僅 non_overshoot 對輪角加端點 roundoff 保護，留在舊輪角與有效目標之間及物理界限內。yaw wrap 不變。

新模式保留 deadband，門檻內可停在目標附近；要求有限命令、合法起始輪角、正步長與合法速率上下限。original 不增加這些參數驗證或 post-clamp，以保留歷史數值行為。

TracedDynamics 只同步新模式實際速率與 effective_target_steering；原模式 trace 欄位不變。PP、WLS、QP、車輛方程、縱向 PID、輪胎參數、FIFO 與全域設定未改。

## 修改檔案

- simulator/dynamic_models.py：新增獨立 non_overshooting_steering_rate；既有函式 AST 不變。
- simulator/dynamics_simulator.py：初始化模式與新模式目標／步長界限。
- research/lmpc/dynamics.py：只同步新模式 trace。
- research/diagnose_steering_actuator.py：獨立固定命令、PP 與原模式回歸工具。
- tests/research/test_steering_actuator.py：17項新測試。
- 本說明文件。

## 驗證結果

研究輸出在 Data/research/steering_actuator/non_overshoot_20261019_001/，詳見 report.md、summary.json 與 figures/。

- 新增17項與既有36項回歸，共53項通過。
- 修改前快照 vs 缺省／顯式original：3000步隨機序列的七維狀態與FIFO bitwise一致。
- 原始A5保存的28660個物理子步重播最大誤差0，沒有重新執行LMPC。
- 四組固定命令的新模式目標過衝／輪角越界均0，FIFO一致，需要時仍達原3.2rad/s速率上限。
- 兩個獨立PP session各完成兩圈。直線yaw RMS約0.148698→0.0160743rad/s、vy RMS約0.0101340→0.000919238m/s；最大|ey|則0.0815376→0.0928440m，不是所有追蹤指標改善。
- 新PP逐物理步與同模式未加trace基類核對誤差0；original PP另與修改前快照逐步一致。
- 339個受保護檔hash不變，包含A5與其他歷史實驗、既有A5診斷、未修改的WLS/QP/PP及全域設定。

## 下一階段 WLS

適合進入各模式獨立資料的受控離線WLS比較；尚未驗證六維模型、12步預測或LMPC持續控制改善。

本次PP採原CentrelinePurePursuit、2.5m/s、lookahead0.6m、control_dt0.1s，使用既有target-speed PID；沒有A2辨識激勵或加速度轉接。兩閉迴路共用設定與起點，後續命令隨狀態不同；固定命令對照另外隔離致動器差異。兩圈PP沒有註冊為LMPC資料庫，不作完整初始化驗收。

後續WLS須固定各模式獨立收集／訓練／驗證方法、必要激勵及真實延伸。不得重標舊A5為non_overshoot。FIFO、速率限制與deadband仍在，命令角仍不一定等於區間有效輪角。舊名義轉向rollout及舊模型假設尚未改動，不能直接開新模式跑LMPC並宣稱預測與plant一致。
