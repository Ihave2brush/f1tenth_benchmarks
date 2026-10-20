# Section III、IV：剩餘問題一次盤點

日期：2026-10-13（Asia/Taipei）。依使用者要求，一次列出實際缺口；只有論文無法完整確認的細節另列，不再逐輪要求選已明列的方法。本次閱讀現有程式與驗收報告，沒有修改控制器或跑模擬。

主規格已定：論文優先；模型辨識只用vx/vy/r、P80、Q_id與h10、Epanechnikov與WLS；正式實驗連續行駛；Safe Set各來源20點、最新兩來源、hard terminal、六維ATV。詳細距離解讀見R0決策，原文記號疑義保留，不繼續列成使用者待選方案。

## A. 確認存在的實作／驗收缺口

| 編號 | 問題 | 確認證據與下一步 |
|---|---|---|
| A1 | 正式runner仍為獨立靜止起跑，未實現新要求的連續PP→LMPC接手 | run_lmpc.py仍sim.reset(start_pose)、stationary_new_experiment=True；之後只執行一個完整圈。需依使用者已定要求接手與持續運行，保留物理狀態／buffer／時間；不用再問是否連續 |
| A2 | 新成功圈尚未更新控制器歷史 | runner terminal_history_fixed=True，database與model在起跑前載入，完成圈雖能保存但沒有自動更新下一圈歷史。需成功圈／continuation資格、資料ready與model池更新 |
| A3 | 終點附近的完整目標未對齊式(8) | QP只實際編入Jλ與input-rate penalty；stage h只作事後報告／凍結描述。所有預測狀態都未過終點時h可為常數，跨線不能直接省略；需完整處理，不能稱式(8)已完全復現 |
| A4 | X/U的索引與額外限制不同於原式 | 原式(8e)state索引為x0..xN−1；現有QP為x1..xN。另有vy/r/heading、s前進限制、保守速度矩形、數值內縮與外部shadow veto。需逐列對齊，區分物理限制、ESP選擇與額外優化约束，不把它們統稱論文限制 |
| A5 | 直接局部模型的獨立多步預測未通過 | validation.json 99個窗口只有18個完成12步，81個中途中斷，passed=false。測試逐步重新辨識，不能直接等同固定ATV rollout；需分解回歸rank／資料激勵／actuator記憶／自由預測誤差，不先判定PID或QP壞掉 |
| A6 | 論文模式完整控制與學習尚無驗收證據 | 尚無paper模式成功整圈、持續學習與改善結果；合成／單元測試和舊hybrid試跑不能代替。需在模型與目標規格對齊後逐階段驗收 |
| A7 | 連續接手時第一個nominal與previous_input仍使用舊靜止條件 | paper_controller首次取第一來源standing歷史，previous_input=np.zeros(2)，而新流程要在實際行進中接手。需以接手的實際狀態與前一個控制對齊初始化；具體seed細節列B2，不將舊seed直接當成通用正確值 |

A1／A7為本次需求變更後需修改的現況，不表示舊靜止實驗當時未依已接受規格執行。

## B. 論文未完整公開或單憑原文無法確認的細節

| 編號 | 無法從論文完整取得的內容 | 已有依據與應處理方式 |
|---|---|---|
| B1 | 跨終點延伸成本及QP處理的完整程式規則 | III定義圈內T−t，V說收集連續兩圈；沒有展開所有suffix index、cost offset與h處理。作者公開碼提供negative suffix／offset證據，但版本與PDF有其他差異，不能當作等價證明。需要明列推導與採用規則 |
| B2 | 第一次控制的完整nominal與末端名義輸入初始化 | 式(10)已給z0=xN的規則；IV-C主要給上一解移位。沒有完整指定沒有上一解時的全horizon seed、連續PP接手的每個初始化細節。以實際接手狀態定義工程初始化，保存理由 |
| B3 | 完整X/U數值及小input-rate penalty權重 | 原文給多面體结构与V的小input-rate cost，没有足够數值重建ESP全部限制与權重。採F1TENTH参数／ESP验证值但明列來源，不假稱論文公布；δ/a命令limits与plant实际限制分開 |
| B4 | F1TENTH底層適配與有限歷史窗口release細節 | 論文使用BARC，沒有F1TENTH PID反解、轉向buffer、弱激勵時rank cutoff，也沒有完整公開本研究的窗口ready判定。沿用真實且因果可用的資料，對適配作獨立驗證；不需無限未來資料 |

其他標記疑義：PDF的weighted norm平方、loss平方與B/C排列已按R0決策採一致解讀；保留來源註記，不把它們重新列為使用者選算法的問題。公開GitHub不是當年實驗commit已證實的版本，因此不能保證在ESP逐項複製BARC性能與收斂圈數。

## C. 已有且可沿用的部分

core幾何／FrenetTracker、同一参考與曲率、實際六維觀測、真實連續PP三圈資料（兩來源加continuation）、D/S/J與Safe Set／Q-function主體、論文式直接回歸與Euler／ATV程式骨架均可沿用。整段規劃域與幾何可用性、有限取樣與連續證明分開；這些已有能力不等於以上A項驗收已完成。

兩類工作不混淆：B類先明列所採用的補足規格；A類按照已定論文方法與使用者要求修正／驗收。R0不是「還有四個演算法要使用者選」，它是採用規格與缺口清單核對。後續只有真正無法由論文及本輪要求決定的細節才提出討論。
