# 論文 Section III、IV 與作者 GitHub 比較

日期：2026-10-13（Asia/Taipei）。範圍：靜態閱讀與公式比較；沒有執行作者程式、修改研究控制器或跑模擬。

## 1. 比較來源與判定規則

主來源：使用者提供的七頁論文 PDF，Section II 的定義、III/IV 式 (4)～(17)、V 與 Table I。論文第5頁 Section V 註腳3直接指向 MPC-Berkeley/barc 的 devel-ugo 分支。

主程式：[BARC LMPC.py 固定版本](https://github.com/MPC-Berkeley/barc/blob/7a481c7044fb85aacd64f2b36df9e29463ea5912/workspace/src/barc/src/ControllersObject/LMPC.py)。commit `7a481c7044fb85aacd64f2b36df9e29463ea5912`，副本與hash在 [來源 manifest](../sources/r0_20261012/manifest.json)。

輔助程式：[RacingLMPC PredictiveModel](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveModel.py)、[PredictiveControllers](https://github.com/urosolia/RacingLMPC/blob/b49c95e7795ef6f343efacedb6b90a69c7855b32/src/fnc/controller/PredictiveControllers.py)。此repository README引用論文，但它是另一份示範實作，不能視為當年BARC實驗完全相同版本。

以下結果描述這些固定快照。沒有確認其與論文實驗當年使用commit一致，也沒有審核所有ROS caller、啟動檔與完整資料。constructor可設定的Q/R/N等值，不能只看類別內程式就斷言當年運行時的數值。

判定分成：核心結構一致、實作補充、直接差異、尚無等價證明。公開程式不是自動覆蓋原文的規格；差異不自動代表論文錯誤或當年實驗沒有使用原方法。

## 2. Section III 對照

| 項目 | 論文 | BARC固定程式 | 判定與影響 |
|---|---|---|---|
| 成功圈狀態／輸入 | III-A式(4)存x/u/T | addTrajectory保存SS/uSS/TimeSS/LapCounter與成本，也保存global狀態與diagnostics | 核心結構一致；successor與輸入時序需按caller核對 |
| 歷史來源圈 | 一般式可用l..j-1；實驗Table I用最近兩圈 | solve按Qfun[0]圈時排序，取lapSelected[:numSS_it]；「保留最新圈」只有註解，實際程式未強制 | 與最近兩圈設定不同；不可忽略排序邏輯 |
| 每圈選點 | 式(5)按weighted squared distance找K個不同時間索引；D只保留s | _SelectPoints用完整六維狀態L1距離找MinNorm，再取前後連續時間窗口；窗口大小由constructor總點數／圈數取得 | 直接差異；中心點與點集可能都不同，連續窗口不保證為原式K最近鄰 |
| D與S資料矩陣 | 選中狀態與同欄後繼 | solve以窗口[:-1]構成終端欄、窗口[1:]構成successor欄 | 核心配對結構一致；額外一點為successor，不是多一個終端点 |
| Local Convex Safe Set | 式(6)λ>=0、sumλ=1、xN=Dλ | simplex有实现，但_LMPC_TermConstr加入六維terminal slack | 有限罰則下不是原文硬終端集合；slack碰巧為零不代表問題定義完全相同 |
| Q-function | 式(7)同一λ對狀態與J做凸組合，最小化Jλ | _LMPC_BuildMatCost將所選Qfun作为λ的線性項 | 同一λ／線性成本主體一致；soft terminal影響Q-function的定義域與原式對應 |
| 圈內成本 | 完成後J=T-t | _ComputeCost從終點往回累加未達終點的每步1，終點及後方初值0 | 圈內主體一致；精確離散終點／首個跨線樣本仍需對齊 |
| 連續跨線資料 | V描述每個iteration收兩個連續圈 | addPoint把下一圈目前量測的s加L，接到前一圈SS，同時更新uSS與suffix cost | 有真正線上延伸機制；不要求無限未來圈，也不是把reset後起點冒充物理後繼 |
| 跨線成本 | III只直接定義至T，V說延伸集合與Q-function | addTrajectory延伸遞減cost；addPoint接suffix；_SelectPoints根據上一個xPred是否跨線加入額外offset | 實作补充；單純signed T-t不足描述完整作者cost策略，不能稱原式完全等價 |
| 最佳化目標 | 式(8)sum h(x)+Jλ，V另加小input-rate cost | cost builder支援Q/R二次成本、dR输入變化、terminal／lane slack罰則與Jλ；未直接編入未知預測狀態的finish indicator | 目標形式有額外部分；Q/R由caller傳入，不能斷言當年一定非零；需對終點邏輯作等價推導 |
| X/U與stage索引 | 多面體X/U，(8e)k=t..t+N-1 | 道路左右halfWidth、δ±0.35、aConstr，stage state不含terminal；lane另有slack | 索引與原式一致，但有限lane slack不同於硬X；數值由環境／caller決定 |
| 終端候選z | 式(10)初始最新來源xN，之後Sλ | constructor先設zVector的s=10；solve後更新Sλ；addTrajectory設第一條LinPoints，但該方法未以最新來源xN重設zVector | 後續推進一致；第一次z是否由其他caller覆寫需核對，不能說整個初始化完全符合式(10) |
| 移位與單次QP | IV-C移位上一解、補zt | solve後LinPoints=[xPred[1:],zVector]，LinInput移位後補successor input組合；先形成ATV，再一次QP | 核心流程一致；不是每控制步必須做兩輪SQP |

BARC 的_selectPoints跨線offset：沒有舊預測或舊預測全在終點前時沿用原cost；已跨線且來源不是最新圈時，加入下一個歷史iteration的起始cost；最新來源使用目前LapTime與上一預測估算剩餘本圈步數。各分支所加量對該來源欄一致，但不同來源可能不同，因此不能只以「λ和為1，所以都是常數」就省略。該public分支按圈時選來源與用iteration+1處理cost也需完整caller／索引核對，不能假設總是在相鄰物理圈。

## 3. Section IV 對照

| 項目 | 論文 | BARC固定程式 | 判定与影響 |
|---|---|---|---|
| 六維模型 | [vx,vy,r,eψ,s,ey] | 六維SS與相同Ai/Ci位置順序 | 核心一致；另有idDelay擴充輸入欄的選項 |
| Frenet Euler | 式(12)以κ(s)及目前速度更新eψ/s/ey | RegressionAndLinearization中的三組位置函數与affine offset使用相同運動學 | 函數主體一致；不是用完整輪胎plant直接替代位置模型 |
| 曲率與梯度 | ∇x f，κ可隨s變 | 作者線性化註明cur constant，depsi_s=0、ds_s=1 | 分段常曲率場景的實作選擇；套到平滑變曲率ESP需重新依原式求導 |
| 位置gradient細節 | 對式(12)求偏導 | BARC ds_ey那一行分母為den*2，而按顯示運動學推導應為den**2 | 存在單行不一致，不能盲抄；尚未確認是否歷史修正或實際使用分支 |
| 速度regressors | 三組五係數，vx用a，vy/r用δ | stateFeatures=[0,1,2]，inputFeatures vx=[1]、lat=[0]，加常數列 | 無額外delay欄時主體一致；steering/input delay會改索引與維度 |
| regression loss | PDF式(13)殘差norm未明列平方 | Q=MᵀWM、b=−MᵀWy，解二次問題；lamb=0 | 明確支持WLS意圖；不是absolute loss，也不是用QP解控制才導致需要回歸QP |
| 辨識選點距離 | weighted state distance，Q=diag(.1,1,1,0,0,0) | ConsiderInput=1，使用[vx,vy,r,δ,a]，縮放(.1,1,1,1,1)，再L1 norm | 直接差異，連最近鄰與權重都可能不同；不能只因前三個scale相同就認定等價 |
| P與h | Table I P80、h10 | N>1時MaxNumPoint=10／來源圈、itUsedSysID=6；RegressionAndLinearization h15；資料依排序選較快圈 | 公開快照不是P80／h10；每圈最多10且正權重不足時更少，不能稱恆有60個有效樣本 |
| Kernel | Epanechnikov K(q)=.75(1−q²)，原式q用weighted distance平方／h | 同一kernel形狀，但引數是上述L1距離／15，只選norm<h的點 | Kernel形狀一致，權重值不等價 |
| A/B/C | 速度用Γ，位置用gradient，位置B零 | 作者填入Ai/Bi/Ci，位置B未賦值保持零；Ci[3/4/5]=eψ/s/ey affine residual | 支持原文列順序修正；函數排列可定稿，不代表每行gradient已正確 |
| Input排列 | II用[δ,a]，式(14)/(16)展示[a,δ] | 程式vx Bi column1、vy/r column0 | 按物理語意排列即可；不應照PDF展示B盲抄到[δ,a] |
| 致動器延遲 | Section III/IV未完整列出實車delay參數與六維閉合性 | steeringDelay、acceleraDelay、idDelay可在regression資料與QP input處理 | 實作補充，需核對實際flags；不能直接聲稱作者完全不考慮底層控制 |

P/h與選點來源的比較僅針對已讀取的function分支，不表示所有作者版本都相同。N、numSS_it、Q/R等由constructor引入，Table I與運行參數一致性需對啟動程式另驗證。

## 4. 另一份 RacingLMPC 的差異

本次輔助快照也使用WLS、Epanechnikov、五係數、Frenet線性化、Sλ、nominal移位、真實addPoint延伸。但：

- PredictiveModel預設每個來源最多7點、h5、帶輸入的縮放L1距離，不是P80/h10的PDF設定。
- addTrajectory按圈長度／圈時排列資料；LMPC選最短的若干歷史圈，不能直接當成「最近兩圈」。
- selectPoints用完整狀態L1找中心後取連續窗口。
- 終端亦有slack，並有根據上一預測是否跨終點的Qfun调整。

因此不把此repository的預設數值补進主計畫，避免同時拼接不同實作版本。

## 5. 對本研究規格的含義

1. 以PDF的明確定義與Table I为主規格：最近兩圈、每來源20個s最近鄰、80辨識點、h10、六維狀態、hard terminal、單步更新ATV QP。數值未公開的項目分列ESP適配。
2. 作者碼支援WLS、state/input物理排列、Sλ推進、nominal移位及真實跨線extension的解讀，這些可以用作推導證據。
3. 不從GitHub直接引入best-lap selection、全狀態L1連續窗口、辨識時input距離、soft terminal/lane、額外tracking cost，然後稱作完全照PDF。
4. IV距離文字疑義、跨線完整cost等價與初次z初始化仍未因讀到public碼而解決；需區分原文數學復現與作者公開工程版本復現。
5. 後續R0仍要討論這些差異，不能以「作者這樣寫」迴避數學與資料因果性。這次比較不批准新的核心選項，也不恢復模擬。

## 6. 已做的查核

三份參考副本SHA256與manifest一致；對照function與其內部運算，沒有執行外部程式。比較結論有明确版本與範圍，沒有宣稱public分支就是當年實驗commit、沒有使用其他作者或非一手文章補齊核心公式。
