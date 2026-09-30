# Continuous laps：進度與接續紀錄

更新日期：2026-09-30

## 目前狀態

Stage 1 corrected research solver 已完成並正式 PASS。兩組 raceline、comparison report 已重新生成；12 項 regression tests 全通過，完整圈與 closure constraint 通過，baseline/upstream hashes 未變。未執行 vehicle simulation，未 commit；Stage 2 尚未開始。

計畫：`docs/research/continuous_laps_plan.md`

## 階段進度

| 階段 | 狀態 | 證據 / 產物 |
|---|---|---|
| Source 調查與規劃 | 完成 | 計畫文件及保存 baseline 的 source/檔案路徑 |
| 1：Corrected closed profile + 四組比較 | PASS／完成 | Data/research/continuous_laps/stage1/comparison.json；12 regression tests PASS |
| 2：Lap tracker | 未開始 | 無 |
| 3：Continuous simulator | 未開始 | 無 |
| 4：Research logger | 未開始 | 無 |
| 5：ESP runner / 三圈驗收 | 未開始 | 無 |

## Git 起始狀態

建立文件前唯讀檢查：

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

1. 讀兩份文件、檢查 Git 與 comparison.json 的 PASS 結果。
2. Stage 1 已全部通過；若使用者指示可開始 Stage 2 lap tracker，目前未開始。
3. corrected_closed_solver 位於 research/closed_velocity_profile.py；僅支援本階段 constant GGV/mu、zero drag、exponent=1 的設定。
4. 重新檢查用 compare --verify-only，不覆寫；只有明確 --replace-stage1 才覆寫兩個具名 research sets 與報告。

Stage 1 命名已固定；finish/crossing、起步時刻與 logging schema 留待後續階段。

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
