# LMPC 完成狀態

截至2026-10-20（Asia/Taipei）。**已建立連續初始化、資料庫、LMPC元件與可追溯的實驗流程；尚無成功LMPC整圈、成功歷史更新或圈時改善證據。** 本頁只維護目前結論，批次數據見 [最新進度](progress_20261020.md)，全歷史見 [實驗總整](../../project/reports/experiment_synthesis_20261020.md)。

## 已完成的能力

| 核心 | 已完成範圍 | 驗收界線 |
|---|---|---|
| 幾何與觀測 | core Frenet中線、曲率、投影／反查、連續進度；LMPC六維狀態與Continuous圈事件接線 | 不代表全道路規劃域或連續時間安全認證 |
| 初始化／資料 | 三圈PP、兩個來源及真實延伸；state／command／FIFO／sample ID／hash可追溯 | PP成功不代表LMPC成功；各plant模式的history分開標示 |
| Safe Set／Q-function | 每來源20點、40欄D/S/J、共享λ、六維hard terminal及Sλ候選推進 | 凸組合不自動保證非線性plant可達 |
| 預測骨架 | 五係數WLS、Frenet Euler、固定12步ATV、首次flying seed與後續移位 | 局部fit小／滿秩／80正權重不代表多步準確 |
| 控制與紀錄 | 每次只施加u0、候選／shadow檢查、原限制驗證、失敗現場與逐物理步紀錄 | 失敗圈不加入成功歷史；重播不是新閉環 |
| 致動器模式 | original預設保留，可選non_overshoot；保存回歸及配對過衝核對 | NO保留FIFO／速率記憶，尚未改善整段預測或持續距離 |
| QP研究 | 固定問題LP／KKT證據、Full／Reduced等價表示與獨立檢查工具 | 固定QP成功不代表後續所有QP收斂；production預設未被實驗替換 |

## 最新驗收結論

| 層次 | 結果與證據範圍 |
|---|---|
| non_overshoot回歸 | 10/19保存53項測試通過；original 3000步bitwise一致，原A5 28660ticks重播誤差0。本次文件整理未重跑 |
| 兩模式PP | 兩圈PP均完成，直線yaw／vy RMS降低，但最大側偏增加，不能說所有性能改善 |
| 獨立預測 | M1平均角改善一步；T1未來真實查詢只屬oracle；多步仍有發散。A7加一維、A8移FIFO均未跑通 |
| QP-D1離線 | 同兩題四種表示共8case，Full兩題成功並通過原限制／KKT；Reduced all也成功，單一故障根因未定 |
| 最新四格閉環 | C00 original/Reduced為23步／6.988937m；C01 original/Full為6步／1.774285m；C10、C11 NO均6步／1.772930m；皆0圈、QP迭代超限 |
| 四格模型／致動器 | 共同6步跨目標17→0、輪角速率反向19→5；yaw預測RMSE約1.07428→1.09255，整段模型未改善 |
| 四格計算時間 | Full共同6步cycle平均約60／62ms，但失敗cycle約443／445ms；完整100ms流程未通過 |
| 新失敗QP | 四格完整runtime現場已保存；尚未另證精確可行性，不把max iterations稱作infeasible |
| 正式WLS補採樣 | 短驗證與original 3m/s資格片段完成；高速資格因heading超限停止，正式train/test均未收集，不是已完成六圈資料集 |
| 持續學習／性能 | 成功LMPC圈自動入歷史、最近兩來源更新、每10圈評估、raceline比較尚未驗收 |

四格固定同一NO history及接手state，只在接手後切actuator／QP；不是每模式各自重新初始化的完整流程比較。舊A5-NO-FULL與四格是分開批次，不能重複計數；C00的23步也不能和原A5的22步直接當單因素比較。

## 目前問題與接續順序

1. **QP數值線：**固定C00成功QP7與C01失敗QP7，再看C10/C11；先核對原限制可行性、KKT與等價矩陣表示。10/19關scaling只解QP2卻讓首QP失敗；Full新QP7仍停止，尚無通用設定。
2. **模型／致動器線：**使用共同6步既有trace拆分command、延遲target、有效輪角、signed資料支持與ATV誤差傳播。過衝已減少，速率飽和、命令反向與外推仍存在，不能只歸因一項。
3. **資料線：**先釐清3m/s採樣資格失敗，再制定可驗收的獨立train/test；不能用target-speed紀錄冒充真正a_cmd或正式激勵資料。
4. 固定資料驗證後，再驗短程、整圈、完整100ms與成功歷史更新；本次不啟動新實驗。

## 如何閱讀舊成果

舊hybrid B3為171步；B2.5另有172步、s約24.0485m但未達34m目標。六維A系列原A5為22步、A9為17步，最新四格C00為23步。它們模型、history、起跑條件不同，均不是成功圈，也不構成跨方法性能排名。

論文PDF、作者公開工程碼與本地方法應分開，見 [方法對照](paper_method.md)。本頁未加總重疊測試、未重跑solver／模擬、未改raw或production。
