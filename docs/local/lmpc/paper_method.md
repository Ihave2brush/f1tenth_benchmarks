# LMPC 論文對照與運作方式

更新：2026-10-20（Asia/Taipei）。下列公式為六維PDF基準，實驗變體另列；不是全部歷史run都使用同一組設定。最新完成狀態見 [completed.md](completed.md)，原作者固定版本與當時行號見 [QP／Safe Set細部對照](archive/20261020_document_cleanup/qp_implementation_comparison_20261019.md)。來源採既有本機論文查核與 [固定快照manifest](sources/r0_20261012/manifest.json)，本次沒有執行作者碼或新增遠端查證。

## 每個控制步怎麼運作

1. 從實際車輛取得六維狀態 $x=[v_x,v_y,r,e_\psi,s,e_y]$，輸入為 $u=[\delta,a]$。core 投影提供 s／ey／epsi，vx/vy/r 由七維 plant 狀態取得。r 是偏航角速度，不是 yaw。
2. 依終端候選 $z_t$ 的 s，在最近兩個成功來源各選20個最近鄰。每欄保存歷史狀態 D、真實 successor S 與成本 J，共40欄；不是全圈只存20點，也不是依當前車輛 s 直接選終端。
3. 第一個 horizon 使用明列的歷史 flying seed；後續把上一個最佳狀態序列移位，末端補 $z_t=S\lambda_{t-1}$。初始候選依式(10)取最新來源的第N個狀態，實際量測始終作 x0。
4. 在各名義點選歷史 transition、做速度 WLS 及 Frenet Euler 線性化，建立整段 ATV：$x_{k+1}=A_kx_k+B_ku_k+C_k$。整段先建立再求解，不在同一 rollout 中依預測狀態重新回歸。
5. 解含 stage cost、歷史終端成本與小 input-rate cost 的問題。hard terminal $x_N=D\lambda$，$\lambda\ge0,\mathbf1^T\lambda=1$；終端位置與成本共用同一 λ。
6. 核對原單位動態／終端殘差、候選幾何與車身，以及原 plant 對真正第一動作的 shadow。只施加 u0，經10個0.01s子步後重新取得實際觀測。
7. 完整成功後才知道新圈 T 與圈內 J=T−t；後續已觀測 prefix 可以接前一成功圈的延伸。當前 runner 尚未完成新成功圈自動加入歷史的持續學習流程。

## Section III：資料、Safe Set、成本與控制

| 論文 | 現行對應 | 驗收／差異 |
|---|---|---|
| (4) 成功圈 x/u/T | `database.py` | 真實 source IDs／時間／後繼配對；圈時插值與離散T分開 |
| (5) 每來源K最近鄰 | `safe_set.py`，距離 $ (s_i-z_s)^2 $ | K=20／來源、最近兩來源；同距離有固定排序，保留真實後繼 |
| (6) Local Convex Safe Set | $D\lambda,\lambda\ge0,\sum\lambda=1$ | 六維hard terminal，沒有terminal slack |
| (7) Q-function | 固定終端x時最小化 $J\lambda$，且 $D\lambda=x$ | 同一λ；LP離線檢查與QP終端線性成本 |
| (8) FTOCP | ATV動態、X/U、h與Jλ，加V節input-rate成本 | X作用於x0..xN−1；xN另由hard terminal限定 |
| (9)～(11) 解、候選推進、第一輸入 | `paper_controller.py`、`run_lmpc.py` | Sλ推進、末端名義控制取successor的u_(i+1)、只施加u0 |

h(x)=1(s<L)，到達當前finish後為0。只有能證明整個可行域的stage status固定時，h才是共同常數；只看 nominal 不足。跨線不確定時現行採凸QP分支並比較完整目標，**不能稱全域永遠只有一次普通QP**。

圈內 J=T−t；資料庫的終點後 signed continuation potential 保留作歷史資料，負值不是負剩餘時間。當前 active-goal 的 terminal cost 在finish之後取0。這是明列的延伸解讀，沒有證明與作者offset策略在所有循環情況完全等價。邊界採原數值gate內表示並重檢動態，不宣稱符號上的全域等價證明。

## Section IV：速度回歸、位置更新與 ATV

N=12、dt=0.1s。辨識 P=80，僅以 vx/vy/r 選點；δ/a 仍在regressor中，不參與距離。

\[
Q_{id}=\mathrm{diag}(0.1,1,1,0,0,0),\quad
 d_i^2=\Delta x^TQ_{id}^TQ_{id}\Delta x
 =0.01(\Delta v_x)^2+(\Delta v_y)^2+(\Delta r)^2.
\]

按 d² 取80個有真實後繼的distinct transition IDs，kernel 引數 q=d²/10；Epanechnikov $w=\tfrac34(1-q^2)$（|q|<1），其餘0。80筆不同時間樣本不代表80個獨立動態方向。

速度三組五係數加權最小平方：

\[
\phi_{v_x}=[v_x,v_y,r,a,1],\quad
\phi_{v_y}=\phi_r=[v_x,v_y,r,\delta,1],\quad
\Gamma^\ell=\arg\min_\Gamma\sum_i w_i(y_i^\ell-\Gamma\phi_i^\ell)^2.
\]

原PDF的distance／loss平方記號有疑義；上述為已定的研究解讀。WLS由作者MᵀWM計算支持，但不宣稱已有正式勘誤。rcond=1e-8為工程數值，paper_direct不用ridge。

式(12)採同一步狀態的Frenet Euler：

\[
\dot s=\frac{v_x\cos e_\psi-v_y\sin e_\psi}{1-\kappa(s)e_y},\quad
 e_\psi^+=e_\psi+dt(r-\kappa\dot s),\quad
 s^+=s+dt\dot s,\quad
 e_y^+=e_y+dt(v_x\sin e_\psi+v_y\cos e_\psi).
\]

κ來自同一core中線。ATV速度block取回歸係數；位置A用Euler函數對六維狀態中央差分，位置B=0；C使名義點的 A*x+B*u+C 與原模型值一致。B依實際輸入順序[δ,a]排列，C依[epsi,s,ey]排列，避免盲抄PDF展示順序。

局部fit誤差小不表示多步穩定。固定ATV獨立預測未通過；實際第一步後的移位nominal也離開kernel支持，見 [歷史進度](archive/20261020_document_cleanup/progress_20261014.md)；最新閉環界線見 [10/20進度](progress_20261020.md)。

## 論文設定與 ESP 工程適配

| 項目 | 採用方式與來源 |
|---|---|
| 初始化 | 論文V節兩圈定速path following；本研究PP參考1.2m/s連跑三圈，只前兩圈為source，第三圈提供真實continuation |
| 連續交接 | 單次reset；保留physical state／buffer／時間／ID／最後PP命令。session s保留，LMPC s=session s−3L |
| 第一次seed | 最新source flying歷史，首名義點換實際觀測，previous_input用最後PP command；論文未完整指定此初次horizon細節 |
| 加速度入口 | 原target-speed plant經逐子步PID反解施加a命令；PP外層a=clip((1.2−v)/dt)，原車模、delay、rate及高速飽和不變 |
| 不可辨識vx gain | dt*cos(beta)，beta=atan2(vy,vx)，靜止beta=0；扣除gain*a後重擬合其他係數。若input可辨識則使用資料擬合值；此為工程假設 |
| rank資格 | 正式設定report_only，保存診斷但不要求滿秩才能呼叫QP；strict保留作診斷。正權重不足仍無法建模 |
| X/U | 速度cap8、vx>=0、heading±.5、ey±.25為實驗選擇；command δ±.4189、a±9.51取simulator值，actual限制由plant保留 |
| 速度多面體 | 32邊內接速度圓，最大徑向保守差約0.4815%；constraint generation仍核對所有facet |
| 其他數值／檢查 | input-rate[.5,.01]、OSQP eps=1e-7、原殘差gate1e-5、footprint margin .05m；全域core認證旗標未改 |

主線沒有接入hybrid物理先驗、九維預測、trust box、terminal slack或兩輪SQP。設定檔保留的legacy residual_ridge／prediction_trust_box欄位不被paper_direct使用；其中sources的standing seed文字也屬舊註記，實際連續runner與最新證據使用flying seed。

## 作者公開碼與 PDF 不能混用的差異

固定BARC快照與輔助RacingLMPC快照支援WLS、六維、Frenet、Sλ、nominal移位與真實addPoint延伸，但存在以下差異：

- BARC按圈時選來源；Safe Set以完整狀態L1找中心後取連續窗口，並有terminal／lane slack，不能直接等同PDF最近兩圈、s最近鄰、hard terminal。
- BARC辨識含input的L1距離、每來源最多10點／使用6來源、h15；RacingLMPC示範為每來源最多7點、h5，均不是PDF P80／h10。
- 作者有signed suffix及依舊預測跨線情況的來源cost offset；不同來源offset不一定相同，不能全部當共同常數消去。
- 作者位置gradient採分段常曲率假設；BARC的ds_ey一行另有den*2與推導den**2不一致，沒有盲抄到ESP。

完整固定快照逐項表保留於 [封存比較](archive/paper_github_comparison_20261013.md)。目前可說已有公式對應與明列適配，不能說已完整復現論文性能、遞迴可行性或學習收斂。

## 10/15 實驗變體與論文差異

| 組別 | 相對基準的改動 | 不能混淆的界線 |
|---|---|---|
| A0／A1 | 原六維模型，PP參考1.2／2.5m/s | 提高初始化速度不表示已改善多步或完成控制 |
| A2 | 2.5m/s PP加受控δ/a擾動，seed20261015 | 明列的新激勵方案，非論文原始初始化數值 |
| A3 | 全歷史最高vx的1.2倍探索上限 | 工程限制，預測遵守不保證實際速度無偏差 |
| A4 | 每stage所選80點的vx與最大絕對命令角1.2倍上限 | 對稱幅度包絡不保證有符號或聯合資料支持 |
| A5／A6 | weighted L1(vx,vy,r,轉向,a)、h15、pooled P80；距離轉向分別為command／actual | 二者regressor仍command；不是原PDF速度Q/h10，也未完整採BARC來源／點數 |
| A5＋歸零 | QP之後將小於0.005rad的第一轉向命令歸零 | 實際施加輸入與最佳化horizon分列；跨servo deadband的效果不由原線性B充分描述 |
| M1 | 離線比較command／actual_start／actual_mean／predicted_mean | measured mean用未來資訊；predicted mean因果，但多步與控制未通過 |
| T1 | 同資料隔離nominal與真實未來查詢，以及teacher／自由預測 | 真實未來查詢是oracle，只作診斷，不能接線部署 |
| A7 | 七維delta_act、五欄平均角WLS、六維終端；delta_next=cmd及連續敏感度近似 | 名義FIFO重播可精確，但QP轉移不精確且未顯式包含FIFO狀態；固定A3命令四組均未改善 |
| A8 | 獨立移除FIFO plant，新三圈PP與資料庫；預測仍六維command WLS | 改plant的對照，非原plant通過；有限速率仍有記憶，不直接等同瞬時轉角模型 |

所有組別尚無成功LMPC圈。A5/A6離線選點單因素比較採`A5_A6_kernel_20261015_002`；`_001`已因同時更改A6回歸而被取代。A7最終短程採`A7_actuator7_20261015_002`；第一輪缺詳細失敗檔。完整結果、資料切分與證據入口見 [10/15進度](archive/20261020_document_cleanup/progress_20261015.md)。

## 10/16～10/20新增變體與目前採用界線

| 組別／工作 | 改動與來源 | 目前結論 |
|---|---|---|
| A2.5／A9 | A2.5僅移除額外δ激勵；A9在其history使用A5選點及探索限制 | 分別3／17步，皆未成功圈；A9不屬只改kernel的單因素比較 |
| original／non_overshoot | 原模式保留預設；NO積分前限制剩餘轉量、有效目標及roundoff保護，FIFO與rate保留 | 已消除本批跨目標，仍有隱藏實際角／FIFO與速率記憶；不是六維預測已閉合 |
| A5-NO／S1 | NO新history接A5；S1只改OSQP scaling0 | NO第二QP失敗、S1首QP失敗；scaling0未成通用設定 |
| QP-D1 | 同原問題比較Full physical／assembled及Reduced saved／all | Full及Reduced all可解兩題，並有KKT；數值表示敏感但唯一根因未定 |
| NO-FULL | NO history與Full physical進入獨立閉環 | 6步後QP7未收斂，不以離線兩題推論整圈 |
| 四格C00～C11 | 同NO history與接手點，actuator×QP；共用raw λ／輸入不投影及原限制／KKT驗收 | 23／6／6／6步，均0圈；同history切plant不等於各模式完整初始化 |
| WLS補採樣 | 真δ/a命令與受控激勵的資格檢查 | original高速資格因heading停止；正式train/test未收集，不宣稱已完成獨立WLS比較 |

**數學問題與數值表示分開：**hard terminal、共享λ、X/U及成本是問題定義；外部縮放、terminal SVD、等式消去與矩陣送入方式是本地表示。Full physical保留全部原物理限制而不作外部縮放／消去；作者公開碼也組裝Full，但另有slack、來源圈／窗口與成本差異，不能因都叫Full就當成同一控制器。

作者模擬與BARC公開碼的terminal／lane slack有有限罰則，λ仍無二次regularization；本地A5／NO／四格保留hard terminal。BARC某wrapper接受MAX_ITER_REACHED不構成原限制已驗收證據，本地沒有據此接受失敗解。原式跨終點、當前goal成本與作者offset的全域等價仍未證明。

production預設未因Full或NO對照自動切換；實驗設定／驗收規則須以該session為準。完整來源、分項差異與未證實假說見 [全實驗總整](../../project/reports/experiment_synthesis_20261020.md)。
