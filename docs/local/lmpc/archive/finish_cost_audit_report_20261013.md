# R0 跨終點成本：實測資料離線對照報告

日期：2026-10-13（Asia/Taipei）。已完成4個ESP實測窗口的離線ATV／OSQP對照；没有執行車輛step、修改控制器或啟動閉迴路模擬。此報告把成本定義、求解方式與模型資格分開，不宣稱完整車輛復現通過。

## 1. 對照資料與做法

來源為已验收的兩個初始化來源與真實連續延伸：`Data/research/lmpc/b2_esp_observed_20261012_001/`。從來源圈2選取距離已記錄終點20、14、8、2個控制步的4個窗口，N12、六維state、各來源20個s最近鄰。

每個窗口固定相同實際initial state、前一control、觀測nominal、PaperLocalModel的12個ATV、D/S欄與現有QP约束。nominal末端是離線anchor候選，並非宣稱此處已经運行了線上式(10)控制器。辨識使用全部既有初始化來源，這是cost-only對照，不是held-out model validation。

沒有將作者整個控制器直接執行；只轉寫固定BARC來源的_SelectPoints成本分支。作者的best-lap／L1窗口／soft terminal沒有混入本輪比较。舊xPred用相同觀測nominal作固定輸入，不宣稱與當年真實舊解一致。

比較五組：

1. `current`：現有signed J與rate成本。
2. `author_cost_only`：在相同D/模型/约束上，只加入作者的每來源跨線cost offset。
3. `paper_h_same_signed_J`：相同signed J，按論文h對finish mask分支求解，隔離stage處理的影響。
4. `paper_h_author_cost_J`：相同作者cost vector，按論文h處理finish mask。
5. `paper_h_clamped_reference_J`：signed suffix改为max(J,0)的單一已完成goal參考；只是比較用，未採为正式循环任务成本。

後三者固定ATV後以LP求各stage進度範圍，證明固定status的stage不列舉；其餘全部finish mask求QP并比較真正stage+terminal+rate成本。嚴格未過線使用1e-4m margin，邊界分類容差1e-6，原始動態／終端／simplex残差門檻1e-5、min λ≥−1e-7；不称符號上无epsilon的全域等價证明。

## 2. 數值結果

表中gap皆使用相同J重算，不拿不同terminal cost問題的objective直接比較。數值約1e-6屬本輪求解精度範圍。

| 距離記錄終點步數 | raw ID | 当前解真實h總和 | 完整mask數 | 当前−literal h，同signed J | 作者−literal h，同作者J | 作者J分支求解耗時ms |
|---|---|---|---|---|---|---|
| 20 | 3916 | 12 | 1 | 3.06e-07 | -6.12e-08 | 39.6 |
| 14 | 3922 | 12 | 1 | 7.44e-07 | 7.04e-07 | 50.7 |
| 8 | 3928 | 7 | 4 | 9.26e-07 | 0 | 282.1 |
| 2 | 3934 | 2 | 1 | -1.89e-07 | -1.01e-07 | 83.0 |

4/4窗口取得全部成本对照；current與author baseline都通過上述physical residual gates，各reference最佳分支亦通過。当前／同signed J literal h的第一個轉向差最大約9.3e-9rad，加速度差極小。兩個終點前窗口所有可行stage都在終點前，h為共同常數；距離8步窗口有4個mask，其中1個infeasible，並沒有省略其他可行mask。

不同作者cost offset在跨線兩窗口造成λ的L1差約2，代表terminal權重重分配；距離8步窗口的predicted r差可達0.1924rad/s，後段轉向差约0.00320rad。第一輸入幾乎相同，不代表整段預測或cost定义一樣。

## 3. 本輪發現的模型限制

4窗口共48個ATV的vx五係數回歸，數值rank只有1或2（rcond=1e-8），不是5。加速度輸入近零、常速資料與局部選點弱激勵，讓目前input影響辨識不足。這是實際rank證據，還不是已定位所有模型失敗的唯一原因。

因此「四窗口第一輸入相同」不能推广为所有速度／加減速／賽道區域的cost等價。R3需要評估特徵與input excitation、選點退化、已識別input gain、數值cutoff和獨立多步預測；不直接切換hybrid或加ridge當成論文方案。

## 4. 求解與實時性限制

首輪iteration limit100k有2窗口未完整收斂；分析資料保留在`r0_finish_cost_audit_20261013_001/`。该首轮已写JSON後最后print格式发生异常，未將其当作成功最终run。

第二輪以相同nearest-terminal one-hot／零deviation和dual重新seed每次求解，analysis iteration limit提升1million，abs/rel tol仍1e-7，4窗口完整。最後追加「literal h+作者J」成第003輪，為本報告主證據。不修改production的100k limit、不放寬殘差門檻。這是對已發生未收斂與新增對照的必要重試。

跨線8步窗口的作者J完整mask求解約282ms，超過100ms，且尚未包含全部辨識／LP／外部驗證時間。本轮没有通過實時性或關閉loop資格。其餘单mask耗时也不能直接当成完整控制耗时。

## 5. 以論文為主的可實作規格

本研究保留Section III／IV明列的狀態、最近两来源、s最近鄰、hard terminal、P/Q/h／WLS及ATV，不混入公開作者分支的best-lap、input L1或slack。

stage h維持依當前decision state定义。可以證明該窗口的h在全部可行解上固定時，單QP加共同常數即可；不可只看nominal。存在finish不確定時，離線基準由分支QP提供明確對照。OSQP仍能解子問題，不能把這個分支流程叫SQP或聲稱永遠單QP／100ms。

跨線terminal延伸以作者cost-only規則作可追溯的循環工程候選：已成功圈的T−t、真實continuation signed potential、當前goal／cycle及上一解跨線信息分别保存。作者方法補足原文未展開的實作，但其与原式所有循环情况的完全等价仍不保证。不得直接將signed potential等同负剩餘時間。

`max(J,0)`只是完成同一goal後cost為0的對照，本輪不把它自動改成正式循環資料庫cost。作者offset也不能略去：不同來源offset可以不同，並非整個QP共同常數。

可進行的下一階段是把這些規格實作成可測的cost policy／offline oracle，先維持production不變；辨識rank与窗口资格须同時驗收。若再進正式控制，需处理性能与數值收敛，不能因离线比較成功就宣稱R3/R4通過。

## 6. R0 狀態與證據

R0「三種成本的實際資料離線對照」已完成，不再因缺使用者資料而停住。stage與signed／作者cost的不同作用已確認，已有可计算的reference與明列工程候選。

嚴格的「原式在所有跨線狀態都等價於單個普通QP」尚未證明，反例顯示不能作一般保證；這不是等待使用者挑公式。完整復現也未通過，須保留這個原文／公開實作边界限制，而不是把四例通過記為全域證明。

主要證據：

- [comparison.json](/data/f1tenth/sim/f1tenth_benchmarks/Data/research/lmpc/r0_finish_cost_audit_20261013_003/comparison.json)
- [可重跑的離線程式](../checks/compare_finish_costs.py)
- [較早的數學反例](cross_finish_resolution_20261013.md)

原始與程式hash、matrix來源、實際selected IDs、各mask狀態、全部inputs/states/λ、solver時間及殘差保存在JSON。没有執行外部作者源码，没有在项目core或controller加入分支演算法。
