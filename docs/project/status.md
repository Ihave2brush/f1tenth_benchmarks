# 基礎、Continuous 與 Frenet 完成總結

核對日期：2026-10-07。

**Continuous 已完成 ESP 連續圈模擬與紀錄／重播驗收；Frenet 已完成六張地圖的幾何、雙向轉換、截面範圍與檢視工具。** 兩者提供後續研究基礎，但目前仍是分開的功能：Continuous 使用既有 Pure Pursuit 與折線圈追蹤，Frenet 尚未接入控制迴路。

行車與幾何批次數值取自保存的驗收 JSON 與報告。本次提交前另重新執行完整研究 pytest；原三圈／五圈行車、全量幾何生成、GUI 與 hosted CI 未重跑。

## 1. 基礎建立到哪裡

原版 benchmark 提供地圖、raceline、車輛動力學、LiDAR、碰撞模型與控制器。我們以 research 模組擴充功能，保留原入口，避免為連續圈而改變原本單圈 benchmark 行為。

| 基礎項目 | 現況 |
|---|---|
| 執行環境 | Python 3.9；研究依賴由 requirements-ci.txt 定義；可使用既有 f1tenth-sim:full Docker 映像 |
| 基準資料 | 原 ESP mu60 與獨立 mu60_closed 配置可分別載入；歷史保護雜湊驗收已保存 |
| 共用核心 | core 轉匯出既有 continuous simulator、圈追蹤與 logger；geometry 提供共用幾何能力 |
| 可重現紀錄 | session 保存輸入、設定、版本、實際樣本與事件；幾何保存 geometry_id、來源及 SHA256 |
| 自動回歸 | tests/research 包含可攜 pytest；CI workflow 使用 Python 3.9／Ubuntu 22.04 |
| 原版隔離驗收 | 已保存原 simulate_laps 結果一致性；新增研究沒有取代原版演算法入口 |

CI 檔存在與本機回歸通過不等於目前 GitHub hosted run 成功。本次未查詢最新遠端 CI 狀態；提交紀錄以 Git 為準。

## 2. Continuous：完成了什麼

Continuous 的目標是一次 reset 後完成多圈，跨起終點不重新初始化車輛，並能事後確認物理狀態的連續性。

| 階段 | 已完成內容 | 主要程式（相對 research/） |
|---|---|---|
| Stage 1 | 閉合速度配置；修正 backward 煞車段長對齊與 settled-lap 切片；閉合段只計一次 | closed_velocity_profile.py |
| Stage 2 | 固定起點的折線弧長追蹤、partial／full、倒退／抖動、跨線時間估計與異常恢復 | lap_tracking.py |
| Stage 3 | 跨圈延續七維 dynamics state 與 steering buffer；碰撞／timeout 明確終止 | continuous_sim.py |
| Stage 4 | 實際 post-step 樣本、動作、時間、圈事件；避免覆寫與重複 finalize | continuous_logging.py |
| Stage 5 | ESP／GlobalPurePursuit 執行入口、N 圈終止、session 驗證與獨立 dynamics replay | run_continuous_pp.py |

Stage 1 的 ESP corrected profile 有 1,153 個點與 1,153 個閉合段，raceline 全長 230.452815 m；預估圈時間 **40.098769 s**。保存報告顯示 combined constraint 零違規，最大殘差約 2.66e−14 m/s²（容差 1e−9）。修正位於 research adapter，沒有修改第三方 helper。

### 實際模擬驗收

保存的三圈驗收包含 3,094 筆樣本，一次 reset；完整圈時間為 41.577166、41.066844、41.063010 s。另已驗證 partial→full、timeout、collision，以及原版 baseline 隔離回歸。

使用者執行的 `manual_run_001` 五圈結果：

| 圈 | 類型 | 圈時間估計（s） |
|---|---|---:|
| 1 | standing start | 41.577166 |
| 2 | flying | 41.066844 |
| 3 | flying | 41.063010 |
| 4 | flying | 41.072549 |
| 5 | flying | 41.083535 |

五圈共 **5,148 筆樣本、一次 reset、零 partial、無碰撞、零投影拒絕／recovery**；verify_session 與獨立動力學重播通過。Flying 平均 **41.071484 s**，比速度配置預估多 0.972715 s（2.4258%）。規劃與模擬含不同模型及控制因素，差異尚未逐段歸因。

圈時間是跨線前後的線性插值估計；物理樣本仍保留實際步進時間，不為精確跨線而回捲狀態。驗收限 ESP、既有控制器與配置，沒有據此批准其他地圖、控制器或實車。

## 3. Frenet：完成了什麼

Frenet 的目標是從原始影像／YAML 建立可重載的參考線與左右射線邊界，提供 x/y↔s/d、定位追蹤與截面可用性查詢。資料支持 ESP、AUT、GBR、MCO、CornerHall 與指定實際 PGM `my_map`。

| 工作 | 已完成內容 |
|---|---|
| 座標與資料契約（A） | map frame、單位、起點／方向、左正右負、版本與實體參考點規則 |
| 幾何生成（B） | 中線、弧長／方向／曲率、逐截面邊界、CSV、來源與保存／重載校驗 |
| 轉換及追蹤（C） | 雙向轉換、精確有效性、投影歧義、連續追蹤、TF 相容資料適配與離線重播 |
| 診斷（D） | ESP 密集數值核對、問題分類與定位；診斷目標完成 |
| 六圖截面範圍 | 輸入 s，回傳道路 d 範圍、採樣 Frenet 區間、拒絕原因與驗證程度 |
| 操作工具 | 同一檢視器切換地圖、點選 x/y、反查 s/d、查截面、顯示候選與限制 |

六張圖均已完成本次 Frenet 轉換與截面介面驗收。ESP 另有完整 C／D 診斷紀錄，各圖的測試方法與證據分別列示；這些差異不影響六圖本次驗收完成。

### 座標與有效範圍

s 是沿目前參考線的弧長，d 是左側為正的法向偏移。閉合圖 s∈[0,L)；CornerHall 為開放主走廊，s∈[0,L]，方向從下端向上再向左，不繞圈、不外插。

一次查詢須通過 occupancy、道路、最近投影／來源一致性、`J=1−kappa*d≥0.2` 與數值往返檢查。道路存在不代表每個位置都有目前介面可接受的 Frenet 表達；有兩個投影候選也不代表一定拒絕。

道路範圍由原 occupancy 精確射線取得；Frenet 區間由逐點採樣取得。選取候選仍須精確查詢，不能把綠色採樣區間當作完整連續域證明。左右邊界 CSV 是法線命中點，不保證可直接相連成物理牆線。

### 六圖保存的截面驗收

以下直接核對 `maps/frenet/<map>/ranges/report.json`：

| 地圖 | 型態 | 截面數 | 採樣查詢 | 局部限制截面 | 往返核對 |
|---|---|---:|---:|---:|---:|
| ESP | 閉合 | 1,194 | 91,791 | 21 | 4,166 |
| AUT | 閉合 | 476 | 39,770 | 27 | 1,661 |
| GBR | 閉合 | 1,013 | 78,381 | 20 | 3,503 |
| MCO | 閉合 | 893 | 69,578 | 48 | 3,131 |
| CornerHall | 開放 | 112 | 9,548 | 8 | 402 |
| my_map | 閉合、實際 PGM | 62 | 2,747 | 0 | 234 |

六圖 `range_interface_passed=true`、`numerical_checks_passed=true`；邊界缺失與完全無接受樣本的截面均為 0。局部拒絕是介面預期回報的限制，不等於整圖驗收失敗；拒絕點數也不是無效面積比例。

my_map 另逐格驗證所選主賽道 **5,204 格中心**，全部通過雙向轉換；7 個孤立 free 格以 outside_track 拒絕。原 YAML 另存，依先前確認將 free_thresh 由 0.25 修正為 0.196，使灰階 205 維持 unknown；來源記錄見 maps/my_map.import.json。

### ESP 額外驗證

- 原五圈及最大 ±0.025 m 人工法線偏差重播各 5,148 筆，全數接受。這是保存樣本的離線測試，沒有加入偏差重新行車。
- C 域樣本為 595 截面、12,612 次來源查詢；595 個中線點皆通過，141 次查詢明確拒絕（127 低 J、14 投影歧義）。
- D 密集檢查共 12,979 截面；117,405 個有效查詢往返通過，438 個來源查詢拒絕。123 個插值區間未達 0.025 m 門檻，因此仍使用精確格子射線。D 診斷完成不改寫舊密集報告的未批准旗標。
- 原 ESP mu60 路線 1,153 點及加密 9,459 點全部轉換通過；最低 J≈0.220551。該截面向內增加 d=0.02 m 會低於門檻而拒絕，顯示路線附近的轉換餘裕仍需驗證。

## 4. 我們做了哪些測試

2026-10-07 提交前以 `bash tests/run_research_tests.sh` 重新驗證：**146 passed、1 deselected，21.61 s**；Python 3.9.25／pytest 8.3.5，`pip check` 通過。my_map 收尾批次另保存 146 passed、1 deselected，21.75 s 的歷史驗收。被排除的是需本機原始 Logs／歷史 manifest 的 `local_archive` 測試，並非失敗；Continuous 歷史批次曾另跑並通過，本次未重跑該項。

| 測試類別 | 已測內容 | 對應 tests/research/ |
|---|---|---|
| 基礎載入／原版 | 必要 import、helper API、raceline 首列與結構、原 mu60 hash、短 headless Pure Pursuit | test_baseline_smoke.py |
| 閉合速度配置 | 非等長段煞車、起點旋轉一致性、完整 N 段約束、closure、幾何保持與避免覆寫 | test_closed_velocity_profile.py |
| 圈追蹤 | 接縫、partial→full、抖動／倒退、時間／速度異常、歧義與 recovery | test_lap_tracking.py |
| Continuous／紀錄 | 跨圈 state／buffer、終止優先序、timeout、碰撞、finalize、重播及毀損偵測 | test_continuous_laps.py |
| 地圖與幾何 | 像素旋轉／半像素、occupancy 門檻、精確射線、變寬中線、保存來源及 hash | test_map_geometry.py |
| Frenet 轉換 | 獨立圓環正反轉換、接縫、低 J、歧義、無效來源與 hint 行為 | test_frenet.py |
| 定位／TF | 跨接縫／倒退、時間及版本、物理點偏移、拒絕不提交與明確重置 | test_frenet_tracking.py |
| 截面／檢視器 | 不跨拒絕樣本合併區間、缺牆、分類、HTTP 路由與非法參數、來源校驗 | test_frenet_inspector.py |
| 開放圖 | 終點不繞圈、有限 s、方向、開放追蹤、範圍 bundle 與混合原因 | test_open_frenet.py |
| 實際 PGM | 灰色 unknown、指定方向、不支援地圖明確失敗 | test_external_map.py |
| 正式候選重載 | 六圖可攜 catalog、gzip 載入、精確截面與保存結果一致 | test_published_tracks.py |

另有三圈／五圈實際模擬、獨立 dynamics replay、六圖 live HTTP 與 GUI 操作的保存驗收。它們不全包含在預設 pytest／CI 中；預設套件不重新生成全部幾何、不跑完整五圈，也不驗證實車。

### 證據位置

正式可重載的 Frenet 候選與摘要位於 [maps/frenet](../../maps/frenet/README.md)。各圖 report、manifest 及 geometry_id 是數值來源；它們不是行車安全批准。

以下路徑從 repository 根目錄定位。五圈分析見[正式驗收報告](reports/manual_run_001_report.md)，圖表與 analysis.json 隨正式文件提供；原始 session 樣本及幾何歷史批次留本機，新 checkout 不一定包含：

| 證據 | 位置 |
|---|---|
| Closed profile 約束／比較 | Data/research/continuous_laps/stage1/comparison.json |
| 三圈、partial、timeout、collision、baseline | Data/research/continuous_laps/stage5/acceptance.json |
| 五圈原始 session | Data/research/continuous_laps/stage5/manual_run_001/ |
| 五圈分析與輸入 hashes | docs/project/reports/figures/manual_run_001/analysis.json |
| ESP C／D 與六圖前置批次 | Data/research/geometry/archive/2026-10-07_before_publication/ |
| 歷史產物位置對照 | Data/research/geometry/archive/location_map.json |
| my_map 數值／格中心／HTTP／GUI／146 項回歸 | Data/research/geometry/external_maps/my_map/import_001/acceptance.json |
| 歷史批次敘述與圖表 | docs/local/research/；入口見 docs/local/README.md |

## 5. 現在的完成界線與下一步

目前可執行 Continuous 多圈實驗，或在六圖上查詢 Frenet 座標與截面限制。三種 s 分別屬於 raceline、Continuous 折線圈追蹤、Frenet 平滑參考線，不能直接互換；ESP 對應長度約為 230.452815、237.329933、236.907498 m。

尚未完成的項目：

- 全道路連續有效域及邊界插值批准；目前 domain_verified、continuous_verified、planning_allowed 皆為 false。
- 使用 d(s) 的路線規劃、整段軌跡與 Cartesian 車身／動態檢查。
- Frenet 接入控制迴路、各圖閉迴路行車與不同控制器的驗收。
- ROS2 Frenet 節點、TF／外參校準及端到端實車部署。
- 超車、對手行為、五路線決策與 LMPC；目前只有目錄預留。

正式文件、五圈精簡證據與本機研究筆記的位置及 Git 納入規則已整理；提交紀錄以 Git 為準。執行方式見[操作指南](usage.md)，接線細節見[介面規格](interfaces.md)，互動查詢見[檢視器](viewer.md)。
