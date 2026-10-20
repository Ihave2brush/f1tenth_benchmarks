# R0 使用者規格決策

日期：2026-10-13（Asia/Taipei）。本文件只更新規格；未修改程式或執行模擬。較早文件若有衝突，以本文件及主計畫最新要求為準。

## 1. 連續實驗

依使用者「如果實驗是連續進行的，那就連續的方式去跑」，正式流程改為連續行駛。PP到LMPC接手及LMPC各圈延續實際車輛狀態、時間與致動器記憶，不把靜止reset作為正式接手方式。

舊資料的獨立靜止實驗保留為歷史。新流程仍需明列最初初始化、接手時間、初次nominal，以及歷史後繼窗口當下可用條件；連續行駛本身不等於能引用尚未收集的未來資料。尚未指定接手必須在第二圈或第三圈哪一個控制樣本。

## 2. 模型辨識選點只用速度與偏航角速度

依本輪「先考慮速度＆偏航角就好」，在前文Section IV的語境下，選點變數固定為 `[vx,vy,r]`；r是偏航角速度rad/s，不是絕對偏航角yaw或航向誤差e_psi。

保留論文Table I的辨識距離矩陣 `Q_id=diag(0.1,1,1,0,0,0)`；δ/a不參與選點距離，s/ey/e_psi亦不參與模型辨識距離。

控制輸入δ/a仍須保留於regressor：vx的五係數輸入為[vx,vy,r,a,1]，vy/r為[vx,vy,r,δ,1]。本決策不表示車輛模型忽略控制效果，也不改變Safe Set按s選點的規格。

距離、kernel與loss的實際採用規格，依使用者後續「就以論文的設計為主」固定於第3節。原文排版疑義保留為來源註記，不再把不同GitHub版本當成同等候選要求使用者選擇。


## 3. 論文優先的辨識規格（D01／D02）

使用者決策：「就以論文的設計為主」。採Section III的weighted squared norm慣例一致解讀Section IV的平方距離記號，並保留Table I數值。這是本研究明列的原文解讀，不宣稱已找到作者當年實驗commit或正式勘誤。

令Δx=x−x_i，以

\[
Q_{id}=\operatorname{diag}(0.1,1,1,0,0,0),\qquad
d_i^2=\Delta x^\top Q_{id}^\top Q_{id}\Delta x
=0.01(\Delta v_x)^2+(\Delta v_y)^2+(\Delta r)^2.
\]

先按d_i²選取80個鄰近且有真實後繼的歷史transition，保持原始樣本ID可追溯。來源池／可用時間依資料契約另外固定，不因此允許未來資料。

Epanechnikov kernel的引數採式(13)所列平方距離除以bandwidth：

\[
q_i=d_i^2/10,\qquad
w_i=\begin{cases}\frac34(1-q_i^2),& |q_i|<1,\\0,&\text{otherwise}.\end{cases}
\]

注意0.1是Q矩陣元素，經QᵀQ後vx平方差係數是0.01；不把它直接當成0.1*(Δvx)²。kernel中的q²是原Epanechnikov形式，不再額外任意改用sqrt(d²)/h或GitHub縮放L1距離。

各通道採局部加權最小平方：

\[
\Gamma^\ell(x)=\arg\min_\Gamma\sum_i w_i\,[y_i^\ell-\Gamma\phi_i^\ell]^2.
\]

φ_vx=[vx,vy,r,a,1]；φ_vy=φ_r=[vx,vy,r,δ,1]；y是真實下一步的对应速度／偏航角速度。WLS解讀由作者公開程式的MᵀWM／MᵀWy運算支持；原PDF殘差norm未明列平方的疑義保留，不把字面absolute loss稱為等價。

簡單手算例：Δvx=0、Δvy=1、Δr=0時，d²=1、q=0.1、w=0.7425。若d²>=10則kernel為0。80是鄰近樣本數，正權重數與regression rank需另記錄，不假設選80筆就足以辨識。

規格狀態：D01／D02的採用方式已定；原文勘誤或歷史版本考證仍未獲證實。數學／數值驗收列入R3，原有多步預測失敗不因規格定稿變成通過。本輪沒有修改程式、調參或跑模擬。

第一項辨識規格討論可結束，下一項為跨終點成本與完整目標（D04／D05）；實驗仍按使用者決策連續行駛。

## 4. 三圈、模擬器限制與10圈路線保存

使用者接受三圈PP準備，但要求確認因果與窗口；physical limits以現有simulator為主；先驗收LMPC成功一圈，再每10圈為一單位。具體規格與未通過項目見 [實驗與儲存規格](experiment_protocol_20261013.md)。目前只寫文件，尚未改程式或執行新run。

## 5. 最新速度上限：8 m/s

使用者確認先以8為主。已核對RaceTrackGenerator.generate_velocity_profile用vehicle_params.max_speed作v_max，GlobalPurePursuit也用同一值截斷速度。正式LMPC實驗上限8，PP參考1.2；simulator的v_max20保留不改。8是上限、不是每段速度目標。生成賽車線与LMPC共享速度cap，但其餘生成器物理假設不一定相同，不能僅因速度上限相同就宣稱完全公平或全域最優。
