# Continuous laps：進度與接續紀錄

更新日期：2026-10-05

## 目前狀態

Stage 1–5 本次 ESP continuous-lap 驗收、研究 regression testing 與 GitHub CI 均已完成。研究功能、報告與 CI 已推送到自己的 fork，並透過 PR #1 合併到 `Ihave2brush/f1tenth_benchmarks:master`。本機 `master` 已同步至 `origin/master` 的 `d1a2200`；同步完成時 tracked/staged 無修改。之後今天的進度、計畫與測試使用說明更新，以及 Docker 測試入口仍是未提交變更；不要將它們誤認為已包含在 d1a2200。今天完成的詳細紀錄見末尾。

目前預設 pytest 為 **47 項 PASS、1 項 local_archive 驗收預設排除**；該歷史 hash 驗收另行執行也 PASS。GitHub 研究分支與合併後 master 的 CI 均已實際通過。先前 Stage 1–5 的 35 項 unittest PASS 保留為歷史驗收，不代表目前 suite 數量。

先前真實 ESP 已完成三個完整圈（41.5771663、41.0668437、41.0630101 s），僅一次 reset，無碰撞／投影異常；所有樣本與 steering buffers 通過獨立 dynamics replay。今天未重跑完整圈或重新生成 raceline，原 mu60、Stage 1 產物、baseline logs、physics 與 upstream helper 保持原樣。

另完成真實 partial→full、timeout、collision 與原 simulate_laps baseline 隔離回歸。保存結果見 `Data/research/continuous_laps/stage5/acceptance.json`。計畫：[continuous_laps_plan.md](continuous_laps_plan.md)；測試與 CI：[regression_testing.md](regression_testing.md)。先前未開始、未 simulation、未 push 等敘述保留為各批歷史狀態，最新狀態以本節與末尾今天的紀錄為準。

## 最新五圈結果與圖表入口

manual_run_001 是使用者手動執行的五圈 session，完成 N=5、partial=0、reset=1、sample_count=5148，無 collision／projection rejection／recovery，verify_session PASS。第 1 圈 41.577166 s；第 2–5 圈 41.066844、41.063010、41.072549、41.083535 s，flying 平均 41.071484 s。前 3 圈樣本與先前三圈驗收完全一致。

- [完整 Markdown 分析報告](manual_run_001_report.md)：包含圖表解釋、重現指令、圈時間與規劃對照。
- [路線總圖／終點放大](figures/manual_run_001/route_overview.png)、[逐圈比較](figures/manual_run_001/routes_by_lap.png)、[全場診斷](figures/manual_run_001/session_diagnostics.png)：均有 PNG／SVG。
- [analysis.json](figures/manual_run_001/analysis.json)：replay 檢查、raceline 幾何距離與輸入 hashes。

Stage 1 closed planned time=40.098769 s；manual flying 平均多 0.972715 s（2.4258%），standing-start 多 1.478397 s（3.6869%）。Closed profile 並非 standing-start 或全域最短圈時間；規劃／模擬模型、PID、steering delay、controller speed cap 與路徑差異尚未逐段歸因。此比較已完成數值核對；沒有藉此調參或宣稱追上規劃時間。

## 階段進度

| 階段 | 狀態 | 證據 / 產物 |
|---|---|---|
| Source 調查與規劃 | 完成 | 計畫文件及保存 baseline 的 source/檔案路徑 |
| 1：Corrected closed profile + 四組比較 | PASS／完成 | Data/research/continuous_laps/stage1/comparison.json；12 regression tests PASS |
| 2：固定 Frenet lap tracker | PASS／完成 | lap_tracking.py；12 tracker tests PASS，含實際 ESP reference 投影 |
| 3：Continuous simulator | PASS／完成 | continuous_sim.py；真實短步進與全場 dynamics replay |
| 4：Research logger | PASS／完成 | continuous_logging.py；exclusive/idempotent finalize、sample/event artifacts |
| 5：ESP runner / 三圈驗收 | PASS／完成 | run_continuous_pp.py；三圈與 baseline 回歸，另 manual_run_001 五圈 replay PASS |
| 最小 regression testing | PASS／完成 | 預設 pytest 47 PASS；本機歷史 hash 驗收另行 PASS |
| GitHub Actions CI | PASS／完成 | push／PR、Python 3.9、必要依賴、pytest；研究分支與 master hosted runs success |
| Fork／PR／本機主分支同步 | 完成 | PR #1 merged；本機 master = origin/master = d1a2200 |

## 歷史 Git 起始狀態

以下是建立文件前的歷史唯讀檢查，不是目前版本狀態；目前狀態見開頭及今天的紀錄：

```text
Branch: overtaking-planner
Untracked: .dockerignore, Dockerfile
Tracked modifications: none
Staged changes: none
```

`.dockerignore` 與 `Dockerfile` 為原有未追蹤檔案，保持原樣。已新增 docs、research 與 tests，未 commit。Data 產物受原 .gitignore 排除。

## 已確認決策

- Baseline 完全保留，研究功能新增而不改原本 episode 行為。
- Initial reset 一次，跨圈保留 dynamics 七維 state 與 steering buffer。
- Collision 結束；N 圈由 runner 結束；timeout 不算 lap complete。
- 第一階段允許重算 minimum-curvature，但要與舊結果比較。
- 保留固定舊幾何 + closed speed 對照，以隔離速度閉合的影響。
- 固定幾何組需幾何完全一致；所有 closed planned time 正確包含 closure segment，且只算一次。
- 五軌跡、opponent、ROS2 暫不實作，只預留 planner 接口。

## 下次接續位置

1. 讀 plan/progress，檢查 Git 及 `Data/research/continuous_laps/stage5/acceptance.json`；本次 Stage 3–5 已通過，不需再視為待實作。
2. 研究功能、測試、CI、圖表／報告與指定小型驗收摘要已保存並合併到自己的 master。Raw samples／scans 及歷史 Logs 仍保留本機；Dockerfile/.dockerignore 與 map_frenet_plan.md 保持未追蹤。新 checkout 不含這些本機檔案。
3. 需重查時用 `--verify-only Data/research/continuous_laps/stage5/esp_three_laps`；唯讀重播資料，不重新跑 simulation。需重跑時使用預設 timestamp session 或全新 --output-dir，已有目錄拒絕。
4. 檢視 manual_run_001_report.md 與路線圖；下一個候選工作是沿 raceline 對齊 planned／commanded／physical speed 及分段時間，目前尚未實作，不先調參。
5. 本次只涵蓋 ESP、既有 GlobalPurePursuit、mu60_closed 與預設 tracker 配置。其他地圖／速度／大偏移、不同 planning scheduler、LMPC／ROS2／opponent／overtaking 都不是本次已完成範圍。
6. 後續從最新 master 建立功能分支，push 後檢查 CI，透過 PR 合併到自己的 master。新的 Frenet trajectory planning／candidate lines／overtaking 行為需補對應測試；現有 CI 不代表整個 upstream 已完整驗證。
7. 使用目前測試入口 `bash tests/run_research_tests.sh`，或 README 的 Python 3.9／requirements-ci.txt 環境。舊 bare Docker unittest 指令保留為歷史紀錄，不作目前完整 suite 的執行方式。

## 後續每批紀錄格式

- 日期 / 階段：
- 實際變更：
- 執行環境與 commands：
- 驗收結果與數值：
- 產物實際路徑：
- 未完成 / 問題 / 下一步：

沒有實際執行的測試需明確寫「未執行」，不得以設計推論代替驗證。

## Stage 1 執行紀錄：2026-09-30

- 環境：pedantic_pare，f1tenth-sim:full，/workspace bind mount。Python 3.9；NumPy 1.22.0、SciPy 1.11.1，其餘版本記於 metadata。
- 主機 git HEAD：c9730789dd0fa85fc18dd4b600c5246e134ebe22。容器沒有 git binary，metadata git_head 為 null；未來可傳 STAGE1_GIT_HEAD。
- 新程式：f1tenth_benchmarks/research/__init__.py、closed_velocity_profile.py；測試：tests/research/test_closed_velocity_profile.py。
- 固定組：Data/racelines/mu60_closed/esp_raceline.csv；metadata：Data/raceline_data/mu60_closed/metadata.json。
- 重算組：Data/racelines/mu60_closed_regenerated/esp_raceline.csv；metadata 與隔離 min-curve：Data/raceline_data/mu60_closed_regenerated/。
- 報告：Data/research/continuous_laps/stage1/comparison.json、comparison.csv。
- 兩組幾何與舊 mu60 前五欄 exact equality；兩組完整數值相同，均 1153 點／段。
- 完整圈長 230.45281503736692 m；closure 0.19985647724542582 m。
- 舊 open time 40.07378694588358 s；舊 speed 補 closure 40.09876900553927 s。
- 新 closed time 40.09880136856941 s；nonclosure 40.07381930891373 s；closure 0.024982059655678228 s。
- 最大 speed 差 0.003609185051879571 m/s；最大 acceleration 差 0.13549931401829474 m/s²。
- baseline、shared centreline/min-curve、params、Logs protected hashes 通過；真實 RaceTrack loader 正確讀全部點。
- strict combined-force residual：新組最大 0.06555353582364232 m/s²，segment index 941，162 段超過 1e-6；closure residual 0。舊 profile 同檢查最大約 2.64e-14，無超差。速度、lateral、absolute longitudinal 通過，不能取代 combined 驗收。
- 初次固定計算因限制殘差拒絕，未寫入。之後 all 寫固定 CSV 後因 git 缺失中止；加入 fallback 與 --resume-incomplete，驗證 CSV exact equality 後只補 metadata，沒有覆寫 CSV。
- 操作：fixed、all、fixed --resume-incomplete、regenerated、compare。最小曲率 solver 約 0.806 s。
- 最終 unittest：7 項通過；compare --verify-only 通過幾何／時間／hash 檢查，仍明確回報 strict_combined_limit_pass=False。測試通過不代表該項限制通過。
- 本次新產物 ownership 已設為主機 kimi（1000:1000）；原檔 ownership 未變。原 tracked 檔無 diff；未 commit。

Docker 重新驗證（不生成、不覆寫）：

```bash
docker exec -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage1_numba_cache -e MPLCONFIGDIR=/tmp/stage1_matplotlib -w /workspace pedantic_pare python -B -m f1tenth_benchmarks.research.closed_velocity_profile compare --verify-only
docker exec -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage1_numba_cache -e MPLCONFIGDIR=/tmp/stage1_matplotlib -w /workspace pedantic_pare python -B -m unittest discover -s tests/research -p test_closed_velocity_profile.py -v
```

全新生成入口為 module 的 all。現有輸出已存在，all/fixed/regenerated 預設拒絕；不可為重跑覆寫 raw 結果。

## Closed solver 問題診斷（2026-09-30）

本次未修改 solver/research 程式或已保存 raceline；僅更新文件。用 Docker 記憶體替換測試，沒有 simulation、geometry optimization 或新 raceline 輸出。

已確認兩項原因：

1. calc_vel_profile.py:414–418：backwards 直接 flip N 個 closed 段長，與 flipped N 個速度點錯位。反轉速度第一步應用原 ds[N-2]，直接 flip 卻用 ds[N-1]（closure）。應將閉合段長先 roll +1 再 flip，或使用等價正確映射。原 ESP 162 個超限全部是減速段；最差 index 941，實際 a=-3.3882868267986166，容許量 3.3227332909749743 左右；用錯的下一段／本段長度比約 1.0197288。
2. calc_vel_profile.py:389–390：backwards 求解後 flip 回原順序，第二個 reverse traversal 的 settled lap 位於返回陣列前半部，原程式卻取後半部。對 closed duplicated profile 應取前 N 個結果。原起點在低曲率高速區，掩蓋了這個錯誤。

只修段長：原起點 residual 約 2.66e-14，但循環平移起點 100/500/941 點時最大加速度約 74.07/41.48/24.64 m/s²，不能採用單一修正。

同時修兩項（記憶體測試）：起始索引 0、100、500、941 的速度恢復原順序後完全一致；圈時間 40.09876900553927 s（末位浮點差），最大 combined residual 2.6645352591003757e-14 m/s²，最大 |a|=5.000832306067765 m/s²。原 protected hashes 不變。

這證明目前檢查的超限不是 closure 算兩次或放寬容差問題，主要來自原 closed helper 的反向索引與圈切片。上述 corrected 數字是記憶體驗證結果，尚未保存為正式 raceline；舊報告仍為 raw helper 結果、strict_combined_limit_pass=False。

建議下一步：research-only closed solver adapter，不修改第三方 helper 或 benchmark；避免 production monkeypatch。保留 raw 兩組，修正版另存；新增非等長段閉合反轉及不同起始索引 invariance 測試，重新執行 Stage 1 驗收。

## Stage 1 修正完成／正式驗收 PASS（2026-09-30）

此段取代前述 raw helper FAIL 的目前狀態；前述保留為診斷歷史。使用者明確授權覆寫兩個 Stage 1 research output 和報告。

實際修改：

- f1tenth_benchmarks/research/closed_velocity_profile.py：新增 corrected_closed_solver；直接沿用 upstream propagation 函式，不 monkeypatch、不改 submodule。只修 backward edge alignment（先 roll +1）與反轉回原順序後的 settled lap slice（前 N）。constant GGV 初始化與原收斂值相同。
- tests/research/test_closed_velocity_profile.py：增加反向非等長 edge 對齊、四起點速度 invariance、全 N 段 combined constraint、closure、幾何 preservation、baseline/upstream hashes、無 monkeypatch。
- docs/research/continuous_laps_progress.md 與 plan：更新驗收／接續狀態。

數值與驗收：

- 兩組均 1153 points / 1153 closed segments；完整 lap length=230.45281503736692 m；closure distance=0.19985647724542582 m。
- corrected fixed 與 regenerated planned time 均 40.09876900553927 s；nonclosure=40.07378694588358 s；closure=0.024982059655678228 s，只計一次。
- 最大 combined residual=2.6645352591003757e-14 m/s²，驗收 tolerance=1e-9；violation count=0；closure residual=0。
- 報告起點 index 0/96/576/864；regression 額外驗證 0/100/500/941。rotate 回 reference 後速度最大差=0，rtol=1e-12、atol=1e-10 m/s。
- 前五欄 exact preservation PASS；regenerated 與 fixed 在本環境完全一致，速度與舊 baseline 也一致。這只代表本 ESP 資料，不假設一般賽道皆相同。
- 舊 open time=40.07378694588358 s；舊 speed 加 closure=40.09876900553927 s。comparison.json 明確分四組，更新 violation count、invariance、geometry differences 與 protected hash manifest。
- Baseline SHA256=61ef20fd634e57befe5c9e3d8086575d507b5eaeffe5fdc54f9ea0f61b1b5a08；原 ESP logs、共享幾何、RaceTrackGenerator 和全部 upstream Python source hash 不變；submodule git status clean。
- Docker 完整 regression：12 tests PASS，無 skip。没有 vehicle simulation，也沒有 Stage 2 變更。

執行：

```bash
docker exec -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage1_numba_cache -e MPLCONFIGDIR=/tmp/stage1_matplotlib -e STAGE1_GIT_HEAD=c9730789dd0fa85fc18dd4b600c5246e134ebe22 -w /workspace pedantic_pare python -B -m f1tenth_benchmarks.research.closed_velocity_profile all --replace-stage1
```

輸出（主機 repository 下）：

- Data/racelines/mu60_closed/esp_raceline.csv
- Data/racelines/mu60_closed_regenerated/esp_raceline.csv
- Data/raceline_data/mu60_closed/metadata.json
- Data/raceline_data/mu60_closed_regenerated/metadata.json 與 esp_min_curve_line.csv
- Data/research/continuous_laps/stage1/comparison.json、comparison.csv

Stage 1 最終 PASS。下一階段可開始，但需要使用者指示；本次未開始。

## Stage 1 版本保存

使用者要求將 Stage 1 成果納入 Git。提交範圍為 research 程式、regression tests、計畫／進度文件，以及兩組 corrected raceline、metadata、隔離 min-curve 與 comparison reports。Data 產物以明確指定路徑 force-add，原 .gitignore 保留。Dockerfile/.dockerignore、baseline mu60、ESP simulation logs 與 upstream submodule 不納入此提交。保存到本機 overtaking-planner 分支；沒有 push，也沒有開始 Stage 2。Commit hash 請以 git log 查詢。


## Stage 2 修訂、實作與驗收（2026-09-30）

- 起始實查：`git status --short` 僅有既存 `.dockerignore`、`Dockerfile` 未追蹤，tracked/staged 無修改；`git log -1 --oneline` 為 `0e89803 Add corrected closed raceline planning and Stage 1 validation`。未找到適用 AGENTS.md 或 workspace PDF。讀 plan/progress、track_utils.py、simulator/f1tenth_sim.py、Stage 1 research 與 tests，並核對實際產物；沒有只依文件假設 Stage 1 完成。
- `compare --verify-only` 實際 PASS，corrected 兩組 strict combined constraint true、零違規、protected hashes 通過。保留 Stage 1 程式、產物、原 mu60、baseline logs、upstream 與未提交 Docker 檔案。
- 原 progress 的分母缺 closure；spline 參數／端點捷徑不等於閉合線段弧長。原 CentreLine 對無 header ESP CSV 使用 skiprows=1 跳過第一數值點。研究 tracker 自行讀完整 maps 中心線，沒有修改原 loader 或 benchmark。
- 實測 maps ESP：1183 點，無重複尾點，第一點 `[0.07687095616231325,-0.0005508749843603435]`；L=237.3299333067795 m，open=237.12943711963652 m，closure=0.20049618714327286 m。smooth ESP 1180 點，L=235.93096368210848 m，closure=0.1999091754315971 m。不能拿 Stage 1 raceline L=230.45281503736692 m 當中心線圈長。
- 產物：`f1tenth_benchmarks/research/lap_tracking.py`、`tests/research/test_lap_tracking.py`；更新兩份 docs。沒有新增依賴、生成 raceline 或寫 simulation log。
- Tracker 使用固定 s=0、closed segment interpolation、signed unwrapped s、local projection、速度/dt/Frenet factor 可行性界限、歧義拒絕、armed/rearm 與 forward threshold crossing。無 reset、無 state/buffer 操作；起點容差內 initial full，其他先 partial；已知初速才分類 standing_start。event 保存線性 crossing estimate 與前後 sample/index/time、diagnostics、epoch；不是精確 crossing state。
- 異常後凍結並 latch recovery_required，caller 明確 recover 才全域重新定位；新 epoch 丟棄中斷圈、先 partial，再完整圈，保留既有 full count，不補計漏圈。unwrapped 只在同 epoch 內連續。
- 第一次 tracker 測試 10/11 通過；恢復測試的浮點 dt=1.0000000000000004 被 max_dt=1 拒絕。加入 1e-12 s 的 max_dt 浮點比較容差後全部通過；未提高進度或力學門檻。
- 最終 24 tests PASS（0.407 s、無 skip）：Stage 1 12 regression，Stage 2 12 tests。Stage 2 涵蓋正常 seam／已知插值、初始化零點與容差、標籤、jitter、reverse、舊門檻不重複、兩個完整圈各自時間、partial→full、非零 e_y、closure／重複尾點、異常凍結與 recovery、非法 time/speed、sampling gap／alias bound、歧義、local branch、numeric CSV 首點與實際 ESP default local search 的兩圈投影序列。ESP 序列是人造 reference samples，不是 simulator simulation。
- `git diff --check` PASS；原 benchmark 行為未改。本次只新增 tracker/tests 並改 docs，未 commit／push。

實際執行 commands（主機 cwd 為 repository；Docker cwd=/workspace，Python 3.9）：

```bash
pwd
git status --short
git log -1 --oneline
rg --files -g 'AGENTS.md' -g '*continuous_laps*' -g '*lap*tracking*' -g '*Centre*' -g '*center*' -g '*centre*' -g '*research*'
find .. -name AGENTS.md -print
rg --files -g '*.pdf'
cat docs/research/continuous_laps_plan.md docs/research/continuous_laps_progress.md
cat f1tenth_benchmarks/utils/track_utils.py
sed -n '1,110p' f1tenth_benchmarks/simulator/f1tenth_sim.py
ls tests/research
head -3 maps/esp_centerline.csv
head -5 Data/smooth_centre_lines/esp_centerline.csv

docker exec -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage1_numba_cache -e MPLCONFIGDIR=/tmp/stage1_matplotlib -w /workspace pedantic_pare python -B -m f1tenth_benchmarks.research.closed_velocity_profile compare --verify-only
docker exec -e PYTHONDONTWRITEBYTECODE=1 -w /workspace pedantic_pare python -B -m unittest discover -s tests/research -p test_lap_tracking.py -v
docker exec -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage1_numba_cache -e MPLCONFIGDIR=/tmp/stage1_matplotlib -w /workspace pedantic_pare python -B -m unittest discover -s tests/research -v

docker exec -e PYTHONDONTWRITEBYTECODE=1 -w /workspace pedantic_pare python -B -c 'import numpy as np; from f1tenth_benchmarks.research.lap_tracking import ContinuousLapTracker; t=ContinuousLapTracker.from_csv("maps/esp_centerline.csv"); print("ESP", len(t.points), "duplicate", t.had_duplicate_endpoint, "L", t.L, "closure", t.lengths[-1], "open", t.starts[-1], "first", t.points[0]); s=ContinuousLapTracker.from_csv("Data/smooth_centre_lines/esp_centerline.csv"); print("smooth ESP", len(s.points), s.L, s.lengths[-1])'
git diff --check
```

主機 `python` 不存在；Docker socket 在 sandbox 中拒絕連線，依既有 docker exec 權限升級後成功，沒有重建容器或安裝依賴。

Stage 3 最小接續：使用 `ContinuousLapTracker.from_csv("maps/esp_centerline.csv")`，初始化完成與每個 post-step 樣本呼叫 `update([x,y,yaw], simulation_time, signed_velocity)`；檢查 valid/reason，記錄 event 與實際 post-step state。runner 只以 completed_full_laps ≥N 停止，partial 不計入 N；異常停止或明確 recover。Stage 3–5 僅更新規格，未實作，未執行原 baseline runner 或完整 continuous-lap simulation。

限制／未解決：配置尚未在 dynamics simulation 校準；Frenet magnification 與 interval 速度界限是假設，並非通用保證。低頻、多圈 alias、未知初始交會 branch、local window 不足可能不可判定。錯誤速度資訊可能使同位置多圈 alias 無法被首尾樣本發現。多邊形 seam 的 outgoing normal 與 incoming tangent 可不連續；目前以 s threshold 判定，未加入幾何線一致性 veto。crossing estimate 是線性估計。Stage 3 初始化時刻與 logger 的實際 lifecycle 仍需依本規格實作驗證。


## 計畫與方向摘要同步（2026-09-30）

更新 plan 標題與開頭摘要，以及本文件的接續方向，清楚區分 Stage 1／2 已完成與 Stage 3–5 待實作。保留既有歷史紀錄、數值、commands 與限制；此次沒有程式、資料或 simulation 變更。文件檢查使用 `git diff --check`；未重跑測試，沿用前述實際 24 tests PASS 紀錄。

## Stage 2 手動驗證環境更新（2026-09-30）

使用者執行舊 `docker exec ... pedantic_pare` 時回報 No such container。實查 `docker ps -a` 無容器，但 `docker images` 仍有 `f1tenth-sim:full`。改用既有映像建立一次性容器並唯讀掛載 repository，實際重跑 Stage 2：12 tests PASS，0.234 s、無 skip。未重建映像、未修改資料或執行 simulation。前述 pedantic_pare commands 保留為歷史執行紀錄；目前手動驗證可使用：

```bash
docker run --rm -e PYTHONDONTWRITEBYTECODE=1 \
  -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace:ro \
  -w /workspace f1tenth-sim:full \
  python -B -m unittest discover \
  -s tests/research -p test_lap_tracking.py -v
```

若執行前述手動 Python 序列，使用 `docker run --rm -i` 取代 `docker exec -i`，加上同一唯讀掛載及映像；`-i` 讓 heredoc 程式透過 stdin 傳入。


## Stage 2 版本保存（2026-09-30）

依使用者指示，Stage 2 成果納入本機 `overtaking-planner` 分支，提交範圍僅包含 `f1tenth_benchmarks/research/lap_tracking.py`、`tests/research/test_lap_tracking.py` 與兩份 continuous_laps 計畫／進度文件。`Dockerfile`、`.dockerignore` 保留未追蹤；不修改 Stage 1、baseline、logs 或 upstream，不 push 遠端。提交 hash 以 `git log -1 --oneline` 查詢。

提交前以一次性容器、唯讀 repository 掛載重新驗證：24 tests PASS（0.403 s、無 skip），含 Stage 1 protected hashes；`git diff --check` PASS。未執行 vehicle simulation，也未開始 Stage 3。

```bash
docker run --rm -e PYTHONDONTWRITEBYTECODE=1 \
  -e NUMBA_CACHE_DIR=/tmp/stage2_numba_cache \
  -e MPLCONFIGDIR=/tmp/stage2_matplotlib \
  -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace:ro \
  -w /workspace f1tenth-sim:full \
  python -B -m unittest discover -s tests/research -v
```


## Stage 3–5 一次整合與實際验收（2026-09-30）

使用者授權一次完成 Stage 3–5。起始 Git 僅有既存 Dockerfile/.dockerignore 未追蹤，HEAD 為 Stage 2 commit 0f382f6；重讀計畫、進度、tracker、原 simulator/dynamics/scan/logger/runner 與 planner。沒有修改原 benchmark、Stage 1／2 程式、params、racelines 或 Logs。

實作產物：

- `f1tenth_benchmarks/research/continuous_sim.py`：獨立研究 lifecycle，沿用原 dynamics/map/scan/四角 collision。default 起點固定中心線首點及切線，暖機四子步後 tracker 初始化；一次 reset、跨圈不 reset。終止原因分開，collision／timeout 當步不產生圈事件；projection_invalid 終止，不自動 recover。state／scan observations 為副本，完全避開原自動保存／destructor profiling。
- `f1tenth_benchmarks/research/continuous_logging.py`：exclusive session directory、full/partial events 與 actual physical samples 分開；十欄 NPY、JSONL、lap CSV、可選 scan NPY、session metadata。保存每步 pre/post state 與 steering buffer，crossing estimate 及前後 sample time/index；explicit idempotent finalize，所有 buffers 清空，例外也 finalized。
- `f1tenth_benchmarks/research/run_continuous_pp.py`：generic planner.plan runner、ESP factory／CLI、N 個完整圈終止、protected hash manifest；verify-only 原 dynamics replay；原 runner 的 temporary cwd baseline 回歸（新 Logs 複製到 research output，原 Logs 完全保留）。
- `tests/research/test_continuous_laps.py`：11 tests，含受控 lifecycle 與真實 ESP dynamics；兩份 docs 同步當前完成狀態、介面、執行方式與限制。

實際驗收結果：

| 案例 | 結果 |
|---|---|
| ESP 三個完整圈 | requested_laps_reached；standing_start 41.57716626647754 s，flying 41.066843716979136 s、41.063010063982915 s |
| 三圈 lifecycle | reset_count=1；total_steps=3094（含暖機），control_steps=3093；simulation_time=123.76000000002641 s，暖機後 elapsed=123.7200000000264 s；最後 crossing estimate=123.74702004743958 s，保留实际 post-step state |
| 三圈 projection | 最大 distance／|e_y|≈0.6278692831633942 m；projection rejections=0；epoch=0；Stage 2 預設門檻未放寬 |
| 三圈 replay | 3094 個 post-step 七維 state、所有 pre/post steering buffers、每個 crossing 前後 sample／alpha／duration 與 N 圈當步停止全部 PASS；scan shape=(3094,1080)，SimLog shape=(3094,10) |
| 原 [0,0,0] 出發 | 先 partial：crossing estimate=0.17661403915322932 s，duration=0.1366140391532293 s；再 full=41.4653227656305 s；N=1 時 full_count=1、partial_count=1、reset_count=1；1042 samples 全部 replay PASS |
| 真實短 timeout | timeout=0.2 s；end_reason=timeout、full_count=0、6 samples、reset_count=1 |
| 真實初始 collision | 明確 out-of-map 起點；end_reason=collision、full_count=0、1 sample、reset_count=1；未呼叫 planner |
| 原 baseline runner | 原 simulate_laps + 原 mu60 在隔離 temporary cwd：Time=41.5200、Steps=1038、Progress=0.9967、LapComplete=True、Collision=False；逐欄匹配保存 baseline |
| 保護 hashes | 四個研究 session 及 baseline 回歸均 PASS，原受保護檔案零修改；Stage 1 protected hash regression 仍通過 |
| 全部 research tests | 35 tests PASS，2.482 s、無 skip；Stage 1=12、Stage 2=12、Stage 3–5=11 |

Tests 涵蓋初始化暖機與 observation copy、crossing 下一步 state/buffer 延續、三個 full events／圈時間／post-step artifacts、partial 不占 N、collision 與 crossing 同步優先終止、>250 秒不算 lap、投影異常停止／不 recover、exclusive/idempotent finalize、planner exception 保存、真實 ESP 短步進／初始 collision、真實 moving-start timeout 與 dynamics replay；修改保存的 steering buffer 會被 verifier 拒絕。Moving-start 初始化需記錄 preceding action=[0,initial_speed]，已由 replay test 驗證。未新增依賴，未重建 Docker 映像。

保存產物（均主機 kimi ownership；現有 .gitignore 排除）：

```text
Data/research/continuous_laps/stage5/
├── acceptance.json                 # 所有案例驗收、source hashes
├── esp_three_laps/                 # session.json, samples/events.jsonl,
│                                  # lap_results.csv, SimLog, ScanLog,
│                                  # protected_hashes.json
├── esp_partial_then_full/          # 相同 schema，無 scan output
├── esp_timeout/                    # actual terminal samples
├── esp_collision/                  # actual initial collision sample
└── baseline_regression/            # regression.json + 隔離 Logs 的副本
```

三圈 ScanLog 約 26 MB，samples.jsonl 約 3.4 MB，NPY 約 244 KB。十欄形狀相容，但本研究 state 是 post-step，action 是 preceding interval；不能與原 pre-step log 混淆，也不能稱為精確 crossing state。

實際 commands（repository cwd；既有 f1tenth-sim:full，container user=1000:1000；沒有固定容器名稱）：

```bash
git status --short --branch
git diff --check

# 第一輪 Stage 3–5 tests：10 tests PASS；補 replay/corruption test 後共 11。
docker run --rm -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage345_numba_cache -e MPLCONFIGDIR=/tmp/stage345_matplotlib -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace:ro -w /workspace f1tenth-sim:full python -B -m unittest discover -s tests/research -p test_continuous_laps.py -v

# 真實三圈；目錄已存在，不可原樣重跑覆寫。
docker run --rm --user 1000:1000 -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage345_numba_cache -e MPLCONFIGDIR=/tmp/stage345_matplotlib -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace -w /workspace f1tenth-sim:full python -B -m f1tenth_benchmarks.research.run_continuous_pp --laps 3 --output-dir Data/research/continuous_laps/stage5/esp_three_laps --save-scans

# 唯讀重播驗證三圈，沒有寫檔／新 simulation。
docker run --rm --user 1000:1000 -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage345_numba_cache -e MPLCONFIGDIR=/tmp/stage345_matplotlib -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace:ro -w /workspace f1tenth-sim:full python -B -m f1tenth_benchmarks.research.run_continuous_pp --verify-only Data/research/continuous_laps/stage5/esp_three_laps

# 原 benchmark 隔離回歸。
docker run --rm --user 1000:1000 -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage345_numba_cache -e MPLCONFIGDIR=/tmp/stage345_matplotlib -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace -w /workspace f1tenth-sim:full python -B -m f1tenth_benchmarks.research.run_continuous_pp --baseline-regression --output-dir Data/research/continuous_laps/stage5/baseline_regression

# 原點出發 partial→full。
docker run --rm --user 1000:1000 -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage345_numba_cache -e MPLCONFIGDIR=/tmp/stage345_matplotlib -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace -w /workspace f1tenth-sim:full python -B -m f1tenth_benchmarks.research.run_continuous_pp --laps 1 --start-pose 0 0 0 --output-dir Data/research/continuous_laps/stage5/esp_partial_then_full

# 最終完整 regression。
docker run --rm --user 1000:1000 -e PYTHONDONTWRITEBYTECODE=1 -e NUMBA_CACHE_DIR=/tmp/stage345_numba_cache -e MPLCONFIGDIR=/tmp/stage345_matplotlib -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace:ro -w /workspace f1tenth-sim:full python -B -m unittest discover -s tests/research -v
```

此外實際以同樣 Docker options 加 `-i`，透過 stdin Python script 呼叫 verify_session 對三圈／partial 產物重播，呼叫 `run_esp_continuous_pp(3, stage5/'esp_timeout', timeout=.2)` 及 `run_esp_continuous_pp(3, stage5/'esp_collision', start_pose=[-1000.,-1000.,0.])`，assert 終止原因與零 full count，將兩組 replay、projection 統計、terminal 短案例、baseline report、protected hashes 與 research source SHA256 一起 exclusive 保存為 acceptance.json，passed=True。

本次尚存限制：只驗收本 ESP/config；保留原四角 sampled collision 模型，未聲稱精確碰撞時刻。terminal outer-step 採保守規則不補計同一步 crossing，projection invalid 直接停止。Frenet sampling／局部唯一性假設、跨多圈 alias 限制仍在；crossing 是線性估計，physical state 不 rewind。原 planned time 不是 simulation pass 判據。未新增 LMPC、ROS2、opponent、overtaking，未改原 benchmark 行為；未 commit／push。


## manual_run_001 五圈分析與路線圖（2026-10-01）

使用者手動 session 實際 requested_full_laps=5，完成 5 個 full laps、0 partial、1 reset，無 collision／projection rejection／recovery。5148 samples 全部通過 verify_session 原 dynamics replay；前 3 圈所有 samples 與既有 esp_three_laps 完全一致。Flying 平均 41.071484 s、母體標準差 0.007741 s、range=0.020525 s（約 0.0500%）；中心線最大 |e_y|=0.627869 m。

新增 `f1tenth_benchmarks/research/plot_continuous_report.py`，以保存 XY/time/events 生成 ESP map overlay、finish detail、各圈 trajectory、lap time／speed／unwrapped progress／offset diagnostics，另存 PNG＋SVG；新增 [manual_run_001 報告](manual_run_001_report.md)。所有圖在 `docs/research/figures/manual_run_001/`，analysis.json 保存輸入 hashes、重播驗證與分析數值。Matplotlib Agg 標準繪圖，使用既有依賴，未新增套件。

額外計算 closed raceline 的全域最近線段無號 Euclidean 距離：mean=0.053763 m、P95=0.122642 m、max=0.379201 m；這不是沿程匹配的 Frenet error，不能與中心線 e_y 或 collision margin 混用。已視覺檢查三張 PNG 的地圖對齊、固定 finish、五圈切片與診斷圖；verify_session PASS、受保護檔案 hashes 未變、文件檢查 git diff --check PASS。此批不重新 simulation，不修改 manual_run_001 原始資料，沒有新增測試或重跑全部 regression；沿用已實跑 35 tests PASS 紀錄。未 commit／push。

實際生成 command（目前 report／figures 已存在，重跑需使用新名稱）：

```bash
docker run --rm --user 1000:1000 \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e NUMBA_CACHE_DIR=/tmp/manual001_numba_cache \
  -e MPLCONFIGDIR=/tmp/manual001_matplotlib \
  -v /data/f1tenth/sim/f1tenth_benchmarks:/workspace \
  -w /workspace f1tenth-sim:full \
  python -B -m f1tenth_benchmarks.research.plot_continuous_report \
  --session-dir Data/research/continuous_laps/stage5/manual_run_001 \
  --report docs/research/manual_run_001_report.md \
  --figures-dir docs/research/figures/manual_run_001
```


## manual_run_001 規劃／實測時間對照（2026-10-01）

依使用者提問，核對 Stage 1 mu60_closed planned_time_s=40.09876900553927 s，將比較表補入 manual_run_001_report.md：standing-start=41.57716626647754 s（+1.478397 s，+3.6869%）；flying 平均=41.07148448651608 s（+0.972715 s，+2.4258%）。說明 minimum-curvature＋closed profile 不等於全域最短圈時間，standing-start 與 closed flying 假設不同；控制器速度 cap、PID、steering delay、路徑偏移及規劃／模擬模型差異尚未逐段歸因。本次僅補文件，沒有重新 simulation 或修改原產物。


## 原計畫／進度文件摘要同步（2026-10-01）

依使用者要求，兩份原文件的開頭同步 manual_run_001 五圈狀態、圖表／報告連結、planned／actual 比較與後续分析方向。Stage 1–5 維持本次 ESP PASS；planned／commanded／physical speed 沿程對比尚未實作。保留歷史紀錄，釐清原 benchmark runner 與新增研究 continuous runner 的差別。本次僅修改兩份文件，連結檢查與 git diff --check PASS；沒有重跑 tests／simulation、調參、commit 或 push。


## Stage 3–5 與分析成果版本保存（2026-10-01）

依使用者要求提交本機 overtaking-planner 分支，不 push。範圍包含 continuous simulator/logger/runner、plot_continuous_report、11 項 integration tests、兩份計畫／進度文件、manual_run_001 Markdown 報告、三組 PNG／SVG 與 analysis.json；明確 force-add stage5 acceptance.json、baseline regression.json，以及五個 session 的 session.json／lap_results.csv／events.jsonl／protected_hashes.json 作為小型驗收證據。原 .gitignore 不變，原 mu60、baseline Logs、Stage 1／2、upstream 與 Dockerfile/.dockerignore 不修改。

約 79 MB 的 raw samples.jsonl、SimLog／ScanLog NPY 和隔離 baseline Logs 副本保留本機，不加入提交。全新 checkout 要執行 --verify-only，需另保留這些 raw artifacts，或用 runner 產生新 session；小型摘要與 figures 可直接查閱。提交 hash 以 git log 查詢，commit message 為 Add continuous simulator, logging, runner and verified lap reports。

提交前實際重新執行唯讀 Docker unittest discover：35 tests PASS，2.564 s、無 skip；acceptance source fingerprints 與五個 session 保護 manifest 的所有原檔 hashes 重新核對 PASS，git diff --check PASS。沒有重新 simulation 或調參。

提交產物格式檢查：清理 Matplotlib SVG 行尾空白，XML 元素 attributes／path geometry（忽略等價空白）保持一致。CSV 保留標準 writer 產生的原始 CRLF，不改 simulation artifacts；staged check 使用 `git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --cached --check`。

## 2026-10-05：Regression testing、GitHub CI 與研究成果整合

### 目標與實作範圍

完成最小侵入式 regression testing 與 CI，保護後續 Frenet trajectory planning、candidate lines 與 overtaking 開發所依賴的研究 baseline。先唯讀檢查 repository、既有 35 項 unittest、submodule、資料路徑與 Docker，再沿用並補強研究測試。沒有替 upstream 建完整測試、修改 production modules、physics／vehicle dynamics、raceline numerical results 或重建 Docker 環境；沒有 CD 或實車部署。

新增檔案：

- `pytest.ini`：限制 discovery 到 tests/research；預設排除 local_archive。
- `requirements-ci.txt`：Python 3.9 必要測試依賴；沿用 baseline numerical versions，pytest 為測試專用新增。
- `tests/research/conftest.py`：測試時使用 repository root cwd，不改 production 路徑。
- `tests/research/test_baseline_smoke.py`：import／helper API、原始及 closed ESP raceline loader、原始 CSV SHA256、真實 PP 10-step headless smoke。
- `docs/research/regression_testing.md`：測試範圍、環境、實際驗證與 CI 說明。
- `.github/workflows/ci.yml`：將先前未提交草稿整理為最小研究 CI。

修改 `test_closed_velocity_profile.py`，增加 finite／正速度與非等長 edges 的解析煞車案例；在所有八個起點驗證正確速度結果。原本需要歷史 Logs 的整份 protected manifest 驗收保留，標記 local_archive，另行執行。移除必要產物缺失時的 skip，避免 fresh checkout 缺資料卻顯示成功。

更新 README 與 `.gitignore`；將既有原始 `Data/racelines/mu60/esp_raceline.csv`、`Data/raceline_data/mu60/params.yaml` 納入 Git。兩個檔案內容未變、未重新生成。其餘大型 raw samples／scans 與 baseline Logs 仍保留本機。

### 測試結果與環境差異修正

| 實際驗證 | 結果 |
|---|---|
| 原 baseline 環境＋pytest | 最初 46 PASS、1 local_archive 預設排除 |
| 無歷史 Logs 的來源副本＋獨立最小 venv | 最初 46 PASS；pip check PASS |
| 本機歷史 protected manifest 驗收 | 單獨 1 PASS |
| 將 reverse-edge 舊 bug 在記憶體重現 | 新 braking regression 在八個起點失敗，確認可攔截 |
| 將 closed-slice 舊 bug 在記憶體重現 | 新 braking regression 在四個起點失敗，確認可攔截 |
| 現代 pip 重現 hosted import 問題 | pip 25.3 下原 editable 安裝重現同樣 7 failed／39 passed |
| 修正後實際 workflow 安裝與測試指令 | 47 PASS、1 deselected，4.59 s；pip check PASS |
| GitHub hosted 研究分支與合併後 master | 均 completed／success |

本地驗證使用既有 f1tenth-sim:full 的 Python 3.9.25，在 `/tmp/f1tenth-ci-validation` 建獨立 venv 與來源副本，repository 唯讀掛載。升級 pip 以重現 hosted 問題只發生於暫存 venv，沒有更新 baseline 映像或 numerical dependencies。最終預設測試包含 baseline smoke 12 項、closed profile 12 項、continuous lifecycle 11 項、lap tracker 12 項；另有 1 項 local_archive。

首輪 hosted workflow 在 job 建立前失敗，原因是 job-level env 使用不允許的 runner.temp context；已將兩個 cache 路徑移到測試 step 的 env。第二輪執行到 pytest，但新版 editable import hook 讓同名外層 submodule 資料夾遮蔽真實 helper package，使 calc_ax_profile 缺失。已在暫存環境重現，改為正常安裝同一 pinned helper，主專案保留 editable，並新增 helper API smoke 防止只驗證 import 成功。沒有 monkeypatch production 或改 solver 數值。

CI 現在是 push／pull_request → recursive submodule checkout → Ubuntu 22.04／Python 3.9 → 必要依賴 → pytest。固定 seed、headless 短步進；不每次跑完整圈、不要求 41.52 s 精確相等、不訓練／繪圖／生成 raceline、不 publish image 或部署。今天沒有重新跑完整圈 simulation。

### GitHub、合併與本機同步

- 建立自己的 fork `Ihave2brush/f1tenth_benchmarks`，原作者 BDEvan5 的 repository 保留為 upstream；自己的 fork 為 origin。
- 提交：8421bbe（最小 tests／CI）、9f0601c（runner context）、48e2dbf（helper 安裝修正）。
- [PR #1](https://github.com/Ihave2brush/f1tenth_benchmarks/pull/1) 已將自己的 overtaking-planner 合併到自己的 master；沒有向原作者合併。Merge commit：d1a2200。
- [合併後 master CI](https://github.com/Ihave2brush/f1tenth_benchmarks/actions/runs/37265498702) 與 [研究分支 CI](https://github.com/Ihave2brush/f1tenth_benchmarks/actions/runs/37265254207) 均已由 GitHub API 查證 success。
- Copilot review 因 quota limit 未完成，不將其列為已完成的 code review；CI 通過只表示已列出的 regression 行為通過。
- 本機曾切到舊 master，落後 origin/master 七個提交，而工作目錄保留新研究內容。逐一比對 59 個變更檔案皆與遠端完全一致後，先備份，再只對齊索引並 fast-forward；未刪除或重寫研究資料。
- 本機同步完成：master = origin/master = d1a2200，ahead／behind 均為 0；overtaking-planner = origin/overtaking-planner = 48e2dbf。Submodule 維持 3d0cd945，無本機修改。
- 同步前備份：`/data/f1tenth/.backups/master-sync-d1a2200-llavsjm2/`，包含檔案、原索引與 SHA256 manifest。這是本機備份，不是 GitHub artifact。
- 原有 `.dockerignore`、`Dockerfile`、`docs/research/map_frenet_plan.md` 保持未追蹤。這份今天的進度／計畫文件更新另待提交與推送。

### 接續與限制

目前可作為研究與協作的 baseline；後續從最新 master 建功能分支，經 tests、CI 與 PR 整合。新成員需 recursive clone submodule；完整歷史 session replay 需另外取得本機 raw artifacts。本次未設定 branch protection，也沒有驗證整個 upstream 或所有地圖／配置。

Frenet trajectory planning、candidate lines、完整 overtaking planner、opponent／ROS2 與 planned／commanded／physical speed 的沿程比較仍未實作。下一階段應先明確選定工作與驗收範圍，再補對應測試。本次進度更新只修改文件，未重新執行 pytest／simulation，也未 commit／push；沿用上述已實際執行與查證的結果。

## 2026-10-05：文件版本釐清與既有 Docker 測試入口

收到檢查摘要指出舊文件的「未提交／未 push」與目前 Git 版本不同，以及裸 image 缺少 pytest。重新實查 HEAD 仍為 d1a2200，研究功能與 CI 已合併；但工作目錄的今天進度／計畫更新實際有未提交修改，不能沿用摘要中的「tracked 無修改」。將歷史 Git 起始狀態明確標示為歷史，更新 README／regression_testing 中過期的「資料待納入下一次提交」「尚未 push」「首次 hosted CI 待驗證」等現況描述，保留歷史執行紀錄。

依使用者提供的重跑摘要，裸 f1tenth-sim:full image 有 23 項 unittest 通過、另兩個測試模組因缺 pytest 無法載入。此結果只代表部分測試成功，不列為完整 suite PASS，也不推翻先前在準備好依賴的環境中得到的 47 PASS。今天 CI／Docker 的差異屬測試環境缺項，沒有據此修改研究功能或降低 assertions。

新增 `tests/run_research_tests.sh`，統一目前本機 Docker 的入口：

```bash
bash tests/run_research_tests.sh
```

使用既有 image、UID/GID 對應與唯讀 repository mount；在一次性容器建立暫存 venv，安裝 requirements-ci.txt，將 pinned helper 的 build inputs 複製到暫存位置後正常安裝，執行 pip check 與完整 pytest。Image 與其原有套件不變；pytest cache 關閉，Numba／Matplotlib／pip cache 放在容器 /tmp。與 CI 統一的是必要依賴清單、helper 安裝方式與 pytest suite，不宣稱容器 OS 或所有間接依賴完全相同。安裝需網路；已有 image 不等於已準備好 pytest 環境。

實際執行新的入口：**47 passed、1 deselected，4.40 s**；`pip check` PASS，Python 3.9.25／pytest 8.3.5。這是本機準備後的完整 suite 結果；沒有重新執行完整圈 simulation，也沒有重建 Docker image。Shell syntax 與 git diff --check 通過。執行後將 pip cache 明確放到 /tmp，避免非 root user 的預設 cache 權限提示。

本批修改 README、regression_testing、今天的進度／計畫說明，新增 Docker 測試入口；尚未 commit／push。原有 Dockerfile、.dockerignore、map_frenet_plan.md 仍未追蹤；新的 tests/run_research_tests.sh 是本批待納入版本控制的檔案，不與這三個舊檔混同。
