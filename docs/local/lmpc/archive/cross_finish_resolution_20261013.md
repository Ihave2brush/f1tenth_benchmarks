# 跨終點最佳化：已定位問題與可驗證的處理方式

日期：2026-10-13（Asia/Taipei）。這份工作回答「是否能處理、是否缺使用者資料」。現有PDF、作者固定版本與本地實作足夠進行技術處理；不用使用者補充數學或另找資料。本次沒有執行車輛模擬或改動控制器。

## 1. 問題已定位，不是OSQP已證實出錯

在固定ATV模型、固定D/S/J、同樣硬終端與輸入限制下，stage indicator依decision state跨線而改變；把它固定在nominal，會改变最优控制。作者的signed延伸potential不自动恢复这个差异。

獨立反例使用兩步scalar affine plant，不是F1TENTH的完整車輛。s0=.9、L=1、s+=s+u、u∈[0,.2]、hard terminal凸集合[1,1.1]；輸入變化成本為u0²+(u1−u0)²，對已到達目標的terminal剩餘成本採0。

| 處理方式 | u0/u1 | 第一預測步s | literal stage+terminal+rate成本 |
|---|---|---|---|
| 凍結stage成本、最小化input rate | .04/.06 | .94，未過線 | 2.002 |
| signed terminal potential −10(sN−1) | .08/.12 | .98，未過線 | 2.008 |
| 依兩個finish分支比較完整目標 | .1/.1 | 1.0，已過線 | 1.01 |

反例證明這些做法不是對所有資料／動態都等價。它不聲稱作者當年實驗失敗，不表示在某個特定局部域永遠有差異，也不驗收本地vehicle model。signed potential實驗的objective與literal objective各自分列。

反例程式：[標準Python active-set檢查](../checks/cross_finish_counterexample.py)。結果：[JSON證據](../checks/cross_finish_counterexample.json)。使用獨立2D凸QP active-set列舉，不是跑OSQP；沒有安裝套件或執行外部下載的程式。before branch用了1e-9數值界限，其最佳解在内部，故此案例不受strict-boundary誤差影響。

## 2. 一個可行的離線參考求解框架

固定每個預測stage的finish status後，h變成已知常數，狀態所處區域用s<L或s>=L限制；ATV、Dλ、Jλ與rate cost仍可形成凸QP。各分支分別求解，再按真正的stage+terminal+rate目標比較，得到該固定模型／資料問題的全局分支比較參考。

這是可以由OSQP求解各子QP的結構，但本次反例並沒有執行OSQP。不需要把OSQP改成SQP，因為此框架是finish分支列舉，不是重複線性化。

對N12，已知x0在終點前，若其他11個stage完全任意，有最多2^11=2048種status組合；terminal state另按Dλ限定。不能承諾100ms內全部解完。若已能证明預測s單調不倒退，才可按第一次跨線index縮成N個分支；不能為降低計算量默加單調约束並稱原式未變。

數值上s<L是strict inequality，需要界限／solver tolerance與邊界規則；原文以s>=L為XF，整數status處理不可留下未標示的epsilon gap。離線驗收须分列邊界內／邊界上／數值容差，不把numerical approximation稱為符號上完全等價。

因此精確literal目標的離線參考求解有路徑，不意味「single ordinary QP」与literal formulation自動等價。普通QP的凸二次目標不能全域精確表示這個nonconvex indicator。

## 3. terminal cost仍需與當前目標一致

本反例terminal點全在XF，所用0是已到達該finish的剩餘成本；實際循環資料的signed suffix是重複任務potential。兩者要分開，才可比較作者工程方式與原式。

對正式車輛的比較要保存：goal finish的位置與cycle、來源T、圈內cost、signed continuation potential、作者每來源offset、實際採用的terminal cost、true finish mask。不能只保存一個名為cost的向量，事後無法辨認目标是本圈還是下一圈。

先以固定D/S/J與literal h驗證「stage凍結是否改解」；再單獨比較terminal延伸定義。不同cost問題不能以各自objective數值比較谁更优；必须用同一reference目標重算。

## 4. 後續技術工作與限制

可完成的工作：建立literal formulation的離線分支參考、與作者signed/offset方案及現有QP逐例對照，記錄改解、可行性、邊界與計算量。這些不需要使用者選公式。

尚未達成的事項：作者單QP工程版本與原式全域等價的proof、當年實驗commit考證、F1TENTH閉迴路／多步模型驗收。不得承諾在精確原式、不增計算量與single QP三者之间无任何tradeoff。

本研究以論文為主；若完整原式與single-QP要求有不可消除差異，應先報告離線對照結果与可審查的方案，再讨论是否接受工程近似，不先把近似塞入主線。

對使用者資料：目前不需要新增資料；若未來要證實當年BARC程式完全相同版本，歷史commit、作者正式勘誤或完整啟動設定會提供額外證據，但本次數學處理不用等待它們，也不要求使用者先取得。
