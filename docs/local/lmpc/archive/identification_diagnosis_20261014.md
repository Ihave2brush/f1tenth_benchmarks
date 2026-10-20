# 模型辨識退化：原因、已修正程式與未通過項目

日期：2026-10-14（Asia/Taipei）。保留ESP、PP參考1.2m/s、P80、Q_id、h10、六維五係數WLS；沒有修改模擬器或PP速度、重收集成功圈、加入ridge／物理先驗、啟動LMPC閉迴路。

## 1. 已確認的原因

完整初始化辨識池為5903個有真實後繼的distinct IDs。只有raw0、raw1的加速度命令幅度大於1e-6：9.51與2.49m/s²，把速度依序由0→.951→1.2。其他指令a約6.66e-15，數值上等於0。

在第二圈99個held-out query anchors，使用全部初始化池或僅第一圈training，所有查詢所選P80樣本的a column都不能在[vx,vy,r,1]之外提供獨立資訊。全池查詢rank histogram：1有63個、2有27個、4有9個；只第一圈：1有54個、2有27個、4有18個。這是縱向input gain不可辨識的證據，並非宣稱所有侧向／偏航自由預測失敗都只有這一個原因。

四個終點附近窗口的selected 80 IDs均不重複，但feature rows高度相同。第一個與第二個案例，vx變化約2.22e-16、vy/r約1e-14；80是不同時間樣本，不代表80筆独立动力資訊。

## 2. 不是資料接錯，也不表示車輛沒有加速能力

全5903 transitions核對原始子步：pre_state=前row physical、post_state=當row physical、command=區間incoming command。施加a區間平均與command差最大1.78e-15；檢查通過。

對四個真實physical state／buffer，各建立原動力學獨立副本，維持原δ，測試a±.1各10個0.01s物理子步。未改live session、未生成成功圈、未加入history或訓練。以原命令重播與真實下一狀態吻合至1e-12。

這些副本的dvx_next/da约.1000，符合0.1s内加速度應有效果；局部minimum-norm回歸的Γ_a約3.26e-15，數值上是0。這並非「模擬器加速度無效」：是資料無法辨識该係數，求解器選到近零的任意最小範數解。

定a的窗口中，把Γ_a增加.1並以intercept扣除.1*mean(a)，對所有80筆量測預測差為0。兩組係數對既有資料一樣好，但在a增加.1時預測速度差.01。這個可重現反例證明單靠該窗口無法選出正確的input gain。

## 3. 底層與初始化控制如何影響資料

原F1TENTH plant的scalar速度由a直接積分，本次加速度適配採 `(1.2-v)/.1`，沒有额外扰动；兩個控制步後v已精確1.2，之後不需a。constant target不是實驗錯誤，但這個理想plant+快速穩定控制的組合與實車資料特性不保證相同。

舊target_speed/PID初始化也閱讀對照：3938rows、32個區間的平均applied a幅度大於1e-6，之後steady scalar speed範圍為0。新acceleration mode為5904rows、2個非零a區間，steady scalar speed範圍亦0。這支持「适配令暫態縮短，原理想plant也精確穩態」；不能只歸咎於新的PID反解，也不能說PID故障。

舊區間平均a只是描述性對照，不冒稱論文的實際command輸入。未取得作者實驗raw logs，因此實車的扰动／command变化属于合理推測，不作为已證实事实。

## 4. 點數與數值選解的對照

僅在離線診斷比較P160／320／640／全部5903，沒有改正式P80。四個案例多数擴到640仍不辨識a；有一例到640納入起步transition而恢復獨立資訊。全5903包含兩個起步樣本，四案例rank5。原因是是否纳入informative transition，不是單純樣本數越多越好；不把global fit直接換進paper local model。

另做相同P80/Q/h/WLS下的numerical-nullspace多解選擇，用第一圈歷史已可辨識channel作seed，不用physical prior或第二圈validation資料。結果：

| 選解 | 99窗口完成12步數 | 原有獨立預測門檻 |
|---|---|---|
| minimum norm | 18 | 不通過 |
| historical seed nullspace | 11 | 不通過 |
| previous fit nullspace | 14 | 不通過 |

後兩種還存在數值nullspace造成的非零SSE变化（尤其previous-fit），報告完整保存。這個方向沒有通过，沒有接入正式控制器，也沒有宣稱只換minimum-norm就解決。不能把資料不可辨識问题靠数值选解隐藏。

## 5. 本次程式修正

- PaperLocalModel逐channel比較含input與不含input的rank（相同WLS绝对threshold），保存input range、獨立資訊norm、fully_identified與input_identifiable。
- 常數或與state共線的控制輸入若不能辨識效果，拋出帶完整source IDs／diagnostics的PaperIdentificationError，不把近零係數冒稱為有效控制模型。
- PaperLocalLMPC在辨識失敗時以failure_kind=unidentifiable_paper_input終止，qp_attempts=0；不先把無控制權的模型送進QP。
- diagnostic_only可以讀取舊minimum-norm fit作離線比較，但不能安裝進正式controller。
- 獨立validator能處理0個有效預測的情況，保存明確拒絕原因而非NaN／崩潰。新的99窗口均在input資格檢查拒絕，passed=false；這是新增誠實gate，不是模型預測已改善。

不因state column退化就誤判任何input都無效：若input仍獨立可識別，診斷會區分「input可辨識」與「全部五係數可辨識」。有限manifold內fit仍不等於off-manifold外推可靠。

## 6. 驗證

50個LMPC測試通過，新增5項涵蓋constant input、varying但state-collinear input、state退化與input独立的區分、未知input时QP不被调用、diagnostic模型不能用于控制；保留既有独立真值回归与六维QP测试。

已保存離線結果：

- [完整可辨識性診斷](/data/f1tenth/sim/f1tenth_benchmarks/Data/research/lmpc/r3_identifiability_20261014_002/diagnosis.json)
- [正式資格gate驗證](/data/f1tenth/sim/f1tenth_benchmarks/Data/research/lmpc/r3_identifiability_gate_20261014_001/validation.json)
- [不採用的多解選擇對照](/data/f1tenth/sim/f1tenth_benchmarks/Data/research/lmpc/r3_wls_tie_selection_20261014_001/comparison.json)

第一份001診斷也保留，002修正將速度／偏航各自input coefficient與a column区分，避免把vy/r的δ gain錯標為a gain。

## 7. 現況與下一步

原因定位與「不能把未知控制效果交给QP」的程式修正完成；模型資格與多步預測仍未通過。三圈資料與Safe Set可保留，不能说因此已经可以开始学会加速。

要恢復局部input effect，需要實際辨識資訊或明列的模型假設；在P80与distance規則不改、沒有新excitation且不用prior的條件下，既有常數input窗口不可能唯一辨識a係數。這是證據指出的資訊缺口，不是要求使用者自己解數學。

下一個資料／介面處理應維持1.2m/s參考，先设计一個可審查的有限辨識方案，明列是否引入微小控制變化、歷史種子约束或其他偏離及來源；未通過独立預測前不進LMPC閉迴路。原文是否足以限定這個理想plant場景的秩不足处理仍需承認沒有公布完整方案，不宣称改用物理prior就是同一論文方法。
