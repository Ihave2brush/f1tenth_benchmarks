# R0 技術規格收尾：交接、資料資格、約束與跨線目標

日期：2026-10-13（Asia/Taipei）。範圍：規格推導、原始資料閱讀及小型數學反例；沒有修改控制器、core或原始資料，也沒有執行模擬。以 [R0決策](r0_decisions_20261013.md) 與 [實驗規格](experiment_protocol_20261013.md) 最新決策為準。


> 後續實際對照已完成：見 [4窗口OSQP報告](finish_cost_audit_report_20261013.md)。下文「下一步比較」已完成；原式全域single-QP等價、模型／性能驗收仍不能宣稱通過。

## 1. 本次結論與結案狀態

- 連續接手的狀態、時間、控制與s契約可以定稿。
- 成功來源、觀測後繼及成本可用時機可以定稿，不要求無限後續圈。
- 模擬器／實驗上限、QP约束與外部檢查可分層定義；實際線性化／內近似與數值驗收在R1/R4處理。
- 跨線時式(8)完整目標與「一次普通QP」的等價還沒有成立。已確認現有signed suffix及nominal h都不能單獨當作證明；此項仍是R0未結案的核心問題。
- 現有多步失敗仍移到R3診斷。沒有把R0規格工作說成控制器或模型通過。

## 2. 連續接手契約

### 2.1 初始資料與接手瞬間

同一session只有最初一次reset。PP跑實體圈1～3，來源A=圈1及圈2真實延伸，來源B=圈2及圈3真實延伸。第三個圈作continuation，不列為第三個初始化source。

圈3完成事件所在的第一個實際post-step樣本，是LMPC初始量測x0。下一個控制區間才切到LMPC；不額外呼叫sim.step當作初始化，不把車速或姿態重設。

必須保留：七維physical state、steering buffer、時間、FrenetTracker狀態、Continuous事件計数、sample ID与最後PP控制。`previous_input`應是最後PP的[δ,a]，不能初始化成[0,0]；這會影響輸入變化成本。

已讀取既有真實資料的範例：第三次跨線row5903、time約590.4s、速度1.2m/s，最后δ約−0.0005015314rad、a近0，buffer為兩筆相同δ；source資料的x0六維須取這個row。範例是原始檔的實際數值，不是未來正式run可硬編碼的值。

### 2.2 s與圈計數

保存原始session s，不覆寫；LMPC local s = session_s−3L，後續每個LMPC圈使用共同平移。既有row5903比3L多0.0679167576m，因此交接狀態的local s是約0.0679，不是精確0。

實際樣本與插值跨線時間分離；保存phase=PP/LMPC、physical_lap與lmpc_lap；LMPC lap001對應實體圈4。跨LMPC圈不reset，將狀態／nominal／candidate／歷史座標一致平移L，κ查詢仍走原core週期幾何。

### 2.3 第一個nominal與後續移位

主線工程初始化：取最新初始化source B的flying圈起點及後續歷史序列作名義seed。名義x[0]更新為實際交接x0；初始terminal candidate仍取該source的xN（論文式10），不能以seed代替實際初始等式。保存seed來源IDs與nominal residual；x0到歷史後續的可行性需R3/R4驗收。

以第一個QP的實際解建立下一步nominal；之後按IV-C移位並补Sλ候選。末端名義輸入若以歷史組合取得，須對應S欄狀態的輸入u_{i+1}，不要直接把原D欄的u_i冒充後繼輸入；若此用途需要額外transition，需另列availability。此處不把歷史凸組合宣稱為任意非線性plant的可行解。

規格已定不代表成功保證。尤其nominal拼接、原actuator記憶、最新歷史與交接x0的差距都需獨立驗收。原文未完整指定first-ever seed，以上是明列工程補足。

## 3. 因果資料資格與來源更新

每個全局transition以ID連結實際(x_t,u_t,x_{t+1})，時間要求完整0.1s控制區間。區分四個狀態：

1. **observed transition**：已執行并取得後繼，可保存與驗證，沒有未來資料。
2. **completed successful source**：整圈完整且验收通過，知道T，具有圈內J=T−t；失敗圈不可加入成功終端來源。
3. **continuation available**：下一圈當前已觀測prefix，可以連續接到前一成功來源；這不需要知道當前圈自己的最終T。
4. **terminal-window ready(z)**：對本次實際候選z，兩個來源各20個原式最近鄰已有所需真實successor，来源／成本規格合法；不足則拒絕，不能擅自補periodic或reset接縫。

新圈完成後即可註冊為成功來源，不必等待再跑完完整一圈才注册。下一圈的observed prefix逐步補continuation；每個控制步只檢查當下候選需要的窗口。最初兩來源已有完整continuation，後續新的來源有自己的完整圈本體，初段窗口通常使用本體。接近終點时下一圈prefix已经逐步觀測，但仍須查實際ID，不以「通常」替代ready判定。

最近兩個成功来源是主規格，不自动以更老或更快的圈替代。若最新來源在某次候選缺後繼，記錄可用範圍与拒絕，不默認更換來源。選點距離只比較已對齊s；每輪保存cutoff、pool版本與selected IDs，動態更新後的選點可追溯。若要對未來可能加入的所有資料也保持最近鄰穩定，另需单调prefix范围界限；当前式(5)只对当时存储的数据定义。

不同用途需不同資格：模型主线暂只使用已驗收的source與其連續observed後綴，不把未完成／未驗收的當前圈本體自动当新成功來源。跨圈延伸相同原始ID只存一次，辨識去重；40个terminal欄按来源保留配對，不因source重叠而误删。

無限延伸不成立：每個控制步只需有限K点及其後繼，nominal／辨識用途需要额外sample時也明列有限範圍。整圈cost、局部successor、模型transition三種availability不能混為一個「整圈ready」條件。

## 4. 約束分層規格

### 4.1 物理plant與本研究選擇

plant参数保留simulator原值：signed v∈[−5,20]、actual steer±0.4189、steer rate±3.2、a_max9.51、v_switch7.319。實驗速度cap8，PP參考1.2。FrenetTracker預設speed15已涵蓋8，不因plant20而放寬。

加速在v>7.319時正向上限為9.51*7.319/v。v=8時約8.70046125m/s²；command9.51不代表实际还能施加9.51。command、requested與applied均保存。轉向命令与实际角、rate与buffer分列。

### 4.2 論文FTOCP約束

x0為實際量測；動態與输入索引k=0..N−1；状态X索引按原(8e)为k=0..N−1；terminal另有xN=Dλ、λ≥0、sumλ=1。不能偷偷把原式state索引改成x1..xN。

终端是否另外检查實驗物理／幾何可用性應记录在外部validation，不冠以原式(8e)。演算法內的hard terminal不加入未授權slack。

δ command與a command入口使用原simulator幅度作候選基本bounds，速度cap8是實驗限制。輸入變化成本的previous_input使用最后实际command。現有vy±.5、r±5、eψ±.5、ey±.25、s前進box、保守速度矩形不是simulator硬體數值，逐项標記／移除或證明用途，不保留其「論文原約束」名稱。

physical speed與(vx,vy)的norm／符號轉換、speed-dependent acceleration、steer rate與delay，不是單一常數polyhedral box就能精確表達。R4必须区分QP內線性／保守約束与原plant shadow精確檢查；任何內近似的保守性需量化，不能說與原plant完全等價。此數值构造屬實作驗收，不新增使用者算法選項。

### 4.3 幾何與實際執行檢查

保持core投影有效性、車身與原occupancy不碰撞、控制时序與版本一致；不將geometry valid当作全車或连续域证明。margin、QP tolerance、input-rate权重皆为工程值，單列source，不歸入物理cap。

實際執行只用第一個控制；原模型shadow結果與QP residual独立記录。拒绝原因区分QP infeasible、模型外推、投影／道路、speed/actuator與数据窗口，不混成「底層PID錯」。

## 5. 跨終點目標：已證實的差異

論文終端集合XF={s>=L}，h(x)=1當s<L，否則0。圈內成功資料J_t=T−t已定；作者公開碼另有signed suffix與跨線offset策略，但不等於PDF完整等價证明。

### 5.1 h不是全域常數，也不是一般凸函數

只要整個候選區域的各stage都確定未過終點，sum h=N，可省略這個共同常數；不能只根據nominal未過線就推定所有QP解也未過線。

反例：L=10、s_A=9、s_B=11、權重0.75。h(0.75s_A+0.25s_B)=h(9.5)=1，而0.75h(s_A)+0.25h(s_B)=0.75，因此違反convexity。跨線stage的精確indicator不能直接當成普通QP的二次／線性目標。

N=3的例子：預測stage進度[9,9.5,10.5]成本2，[9,9.25,9.5]成本3。凍結同一nominal count會遺失這個差異。這是符號目標差異，不是OSQP調容差可修復的問題。

### 5.2 signed suffix不等於已到終點後的剩餘時間

T=10、t=12時原signed potential=T−t=−2；若目標是已到達的同一個finish，剩餘到達時間是0。−2是重複任務延伸potential，不是負的行駛時間。

因此数据需区分`lap_cost`（圈內T−t）、`continuation_potential`（有號延伸）与`terminal_cost_for_active_goal`（供本次FTOCP真正使用的J）。最後一個仍須推導，不能把存储的potential直接叫式(7)已到達後的cost。

### 5.3 目前唯一尚未結案的原文核心目標問題

需要決定一個可驗證的完整實作：如何從連續data建立当前finish／下一finish的terminal cost，並與sum h相容，同时保持聲稱的求解形式。

作者的_SelectPoints參照上一预测跨線情況調整每來源cost，不可直接宣稱精確重現依当前decision state定義的indicator。固定分支一次QP、跨線分支多QP、混合整數／其他方式各有不同数学含义；本次不擅自把其中一個寫成「完全相同的單QP」。

下一步技術核對应在明確边界例上比較三者：原式stage+terminal目標、作者signed/offset实现、现有plain signed Jλ+rate。原文缺完整proof或程式規則時，承認未證明；不再要求使用者在數學記號之間選擇。若要採近似模式，须另立明示偏離，不能以此結案原文等價要求。

## 6. 本次實際檢查與後續工作

閱讀了原始row5903與圈事件，核對真實交接狀態／指令／buffer／s；以host Python計算上述两个反例，結果一致；沒有作動力學新試跑。

交接、因果資料與約束語意可作为R1/R2/R4實作依據；尚未驗收controller。跨線目標仍未完成等價推導，所以R0仍未整體通過。下一步聚焦本節的邊界例與作者cost策略，不重复討論已定的速度、連續行駛或辨識參數。
