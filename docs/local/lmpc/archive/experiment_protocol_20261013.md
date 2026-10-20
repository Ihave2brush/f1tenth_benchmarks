# Track B：連續接手、模擬器限制與路線保存規格

日期：2026-10-13（Asia/Taipei）。依使用者三項回覆制定；本次僅更新規劃，不修改控制器、重收集資料或跑模擬。本文件取代獨立靜止LMPC正式實驗的選擇；最新速度決策仍採8，並補充主計畫與R0決策。


> 最新使用者修正：實驗速度上限採8 m/s，與原賽車線生成器與PP的vehicle_params.max_speed一致；simulator v_max=20不修改。以下plant數值表描述底層設定，不代表允許LMPC跑到20。速度相關窗口驗收按8重新核對；不因plant20就把tracker改到20。

## 1. 論文方法與三圈初始化

論文第5頁 Section V 明確描述：以兩圈定速path following初始化；每個iteration收集兩個連續圈的資料，使Local Safe Set／Q-function也定義在終點之後；後續成功閉迴路圈更新控制器。

原文沒有要求「一定PP三圈後才接手」。本研究第三個PP圈是為了在切換前準備第二來源的真實continuation，不把它冒稱第三個初始化source。這是明列的收集時序選擇。

正式流程：

1. 同一session起點初始化一次，PP以1.2m/s參考連續跑圈1、圈2、圈3。
2. 初始化來源A為圈1，使用圈2的真實量測作延伸；來源B為圈2，使用圈3的真實量測作延伸。
3. 圈3完成的第一個實際post-step樣本作為交接觀測，下一個控制區間由LMPC輸出。時間、七維physical state、steering buffer、實際轉向角及前一個PP控制指令全部延續，不reset、不另行把車速改成0。
4. 此後LMPC各圈連續。PP圈數與LMPC圈數分别標記；LMPC第一圈是session的實體第4圈。

資料結構仍有兩個初始化source；不需要lap_002_periodic.npz，也不把第三圈拿來頂替沒有觀測的後繼。第三圈亦可以是辨識池內有真實後繼的量測，但其model池用途與source標籤分列。

### 三圈是否還有問題

在資料因果性上可行：到交接時，來源A／B的整圈及各一個連續延伸圈都已觀測。既有observed資料庫確實按此結構構建，並通過原有B2窗口驗收。

仍須驗證，不是額外的使用者演算法選擇：

- 原驗收只對當時設定的候選範圍掃描，依本次8m/s實驗上限，按N、可達範圍、候選推進與實際窗口重新驗收；有一整圈延伸可供查詢，不代表所有新窗口已測過。
- 第一次nominal與previous_input要配合行進中的實際交接，不沿用舊standing seed及零輸入；z0依論文式(10)，但最新歷史與實際交接狀態是否可行須實測。
- 所有s保持同一core參考。可保存session s及以交接終點3L為基準的LMPC s；第一個post-step樣本可能已略過線，不憑空插入精確s=0的物理狀態。圈計時保留插值事件。
- 新LMPC圈的觀測即時保存，可連續延伸前圈；只有完整成功後才有該新圈自己的最終cost。加入成功來源与取得充足successor窗口分開，不引用未來、不要求無限圈。
- 第三圈不作第三個初始化source，故此處的「最近兩圈」指可用的兩個初始化来源。後續成功来源替換的ready规则必须明列，不能在未有延伸时强制取一个缺後繼的source。
- 模型多步驗證失敗、跨線目標未完整對齊仍是控制實作缺口。三圈解決的是初始化資料的實測延伸，不能保證LMPC求解與真實成功。

原有三圈檔案可作離線數學／資料驗證。要做到真實連續接手，需在同一live session延續，或明確重播到交接的完整物理狀態與buffer；僅載入兩個NPZ再reset不符合此正式流程。本規格預設新正式run從PP到LMPC一路不中斷。

## 2. 限制以目前模擬器為主

來源為 `params/simulator_params.yaml` 與原 dynamic_models.py，保存resolved params與hash，不從另一份vehicle_params拿不同物理上限。

| 量 | 現有值／規則 |
|---|---|
| signed physical speed v | −5～20 m/s |
| 實際轉向角 | −0.4189～0.4189 rad |
| 轉向角速度 | −3.2～3.2 rad/s |
| 最大制動／加速幅度 | a_max=9.51 m/s² |
| 正向加速度的高速限制 | v>7.319時 a_max_positive=9.51*7.319/v，低速為9.51 |
| 物理子步 | 0.01 s |
| LMPC控制區間 | 保留論文0.1s；每區間10個原物理子步 |
| 車身尺寸 | length0.58m、width0.31m |
| 致動器 | 原PID、兩物理子步steering buffer與限制保留 |

本次實驗上限為8m/s；20只是保留的plant原始v_max，不要求改動simulator，也不允許本次控制以20作上限。正式輸入語意保持δ/a，原plant仍施加其speed-dependent限制。

速度部分：現load_b3上限8与最新實驗決策一致，core FrenetTracker預設15涵蓋8，不因plant上限20而放寬。QP的steering limit0.4與plant0.4189仍需按物理限制決策另核對；保留時間／投影檢查，不直接解除或全域改驗收旗標。

speed-dependent acceleration、body velocity轉physical speed、轉向delay/rate無法只以δ/a兩個固定box完整表示；原plant保留真實限制，QP使用哪些線性／保守约束需要推導与独立核对。不能把a≤9.51当作所有速度下都实际可施加。

原QP vy/r/heading/s額外box、vx>0的前進条件、margin與shadow檢查不是simulator物理數值；應拆開記錄，逐项檢查是否必要、是否改变論文問題，不能默稱「全部照模擬器」。道路／車身不碰撞仍是實驗成功条件；physical limits不提供Frenet連續域或整車安全證明。

input-rate cost權重與OSQP容差不是simulator限制，也未因本次物理限制決策自動改成作者未公開數值；保留明列數值選擇與后续驗收。

## 3. 驗收一圈，再以10圈為一單位

- PP1～3圈完成後直接接LMPC。
- LMPC lap001作首圈可行性驗收：實際完整圈、無碰撞／投影失效、模型／QP資料完整、輸入對齊及物理重播。可保存成功資料，但沒有成功證據前不宣稱學會。
- 通過後，第一個10圈單位為LMPC lap002～011，下一個為012～021，以此類推。單位是報告／checkpoint邊界，圈與圈不reset、不重設歷史。
- checkpoint只保存狀態与結果，不假設車輛停在起點。若實驗必須停下，記錄中斷與後續是否新session，不能把重啟說成物理不中斷。
- 失敗則保存failed/incomplete狀態与原因，不能改標成功、污染终端成功來源或掩蓋資料缺口。

## 4. 每圈學到的路線怎麼保存

採用獨立run目錄，規劃位置：`Data/research/lmpc/runs/<run_id>/`。下列為待實作的格式，不宣稱目前runner已有。

| 檔案／目錄 | 內容與用途 |
|---|---|
| session.json、resolved_config.yaml、manifest.json | map／geometry_id／code與source hashes、參數、一次reset、phase／圈標籤、結束原因、資料版本 |
| raw/samples.jsonl | 連續真實post-step Cartesian／七維物理／六維Frenet、時間、phase、sample_id、buffer与command；逐步保存避免中断遗失 |
| raw/physics_ticks.jsonl、events.jsonl | 真實致動器子步、跨線事件／插值時間，支持獨立重播与successor追溯 |
| raw/controls.jsonl | 每步預測x/u、λ、選點來源、辨識資料、QP結果、實際误差；與实际行驶线明确區分 |
| laps/pp_001.npz～pp_003.npz | 三個原始PP圈；來源A/B與continuation角色在manifest標記 |
| laps/lmpc_001.npz、lmpc_002.npz… | 每圈真實軌跡、速度、輸入、物理狀態、時間、sample IDs、完成資格与cost；成功與失败區分 |
| routes/lmpc_001_actual.csv… | 每圈實際跑出的路線：sample_id、time、s_session、s_lap、X/Y、yaw、vx/vy/r、eψ/ey、δ/a；可畫圖与比较，欄位標明incoming/outgoing命令時序 |
| history/index.json、checkpoints/… | 成功source、延伸raw ID範圍、ready狀態、D/S/J建構版本，以及每單位checkpoint；避免要求額外複製無限尾段 |
| reports/batch_001.json… | 10圈圈時、誤差、速度、成功狀態與耗時，附每圈／最快成功圈路徑与hash |

重點：完整原始資料一直保存。單圈NPZ保存該圈本體；continuation依index引用下一圈已觀測樣本，不需把同一批量測重複存成無來源的新資料。延伸索引更新與成本策略使用version/hash，保留舊snapshot可重現。

真實Cartesian X/Y直接取自plant，不把中線投影點冒充車輛路線。每圈CSV即使保存得像waypoints，也只是實際閉迴路路線，不稱全域最佳線。控制器每步的預測轨迹另存controls，不取代實際路線。

每單位報告保存最快「成功且驗收」圈的引用／hash，不把十圈平均位置直接叫最優路線。若後續要輸出可供其他planner追蹤的等距waypoints，再從指定成功圈另外重採樣並驗證閉合、曲率、速度與致動器可行性；原始路線不覆寫，與外部生成賽車線保留不同來源。

Checkpoint為恢復／診斷保存physical state、buffer、時間、tracker／圈計数／s原點、controller candidate／nominal／previous input、歷史與模型資料版本；恢復同樣數值狀態須另驗收，不先承諾bitwise continuity。

## 5. 本輪已決定與仍需技術處理

使用者已決定：接受三圈PP準備、正式連續接手；physical limits以simulator為主；成功一圈後以10圈為單位。路線保存採每圈raw／NPZ／CSV、每單位摘要与历史索引，為本輪完整可檢視規劃。

不再要求使用者選已公開的論文方法。跨線完整目標、連續交接初始化、模型失敗、QP物理约束表示与資料ready由技術工作依主規格處理；真正无法由原文與本輪要求決定的必要差異才另外說明。此次僅規劃，儲存与控制接線待後續階段實作驗收。
