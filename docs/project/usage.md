# 操作指南

所有指令從 repository 根目錄執行。原版 benchmark 入口見根目錄 README；
以下只說明新增研究功能。資料產物通常不在 Git，下載專案不代表已有歷史 session。

完成狀態與保存的測試結果見[完成總結](CHANGELOG.md)。以下指令用於重現操作，不代表本次文件整理有重新執行。

## 環境與測試

使用 Python 3.9；建議在自己的 virtualenv 內安裝：

```bash
git submodule update --init --recursive
python -m pip install -r requirements-ci.txt
python -m pip install --no-deps ./trajectory_planning_helpers
python -m pip install --no-deps -e .
python -m pip check
pytest -p no:cacheprovider
```

helper 採一般安裝，避免 editable namespace 衝突。本機不需要 ROS2。
既有 `f1tenth-sim:full` 映像可執行 `bash tests/run_research_tests.sh`；
腳本使用暫存 venv 安裝依賴並唯讀掛載專案，需要 Docker 與下載依賴的網路。
預設回歸不包含歷史 local_archive 與全量五圈重播。

## 連續圈與速度配置

若尚無 mu60_closed 研究資料集，先產生獨立速度配置：

```bash
python -B -m f1tenth_benchmarks.research.closed_velocity_profile all
```

需要原 ESP mu60 raceline／參數與 helper；已存在的輸出不覆寫。
檢查既有配置：

```bash
python -B -m f1tenth_benchmarks.research.closed_velocity_profile compare --verify-only
```

```bash
python -B -m f1tenth_benchmarks.research.run_continuous_pp \
  --laps 3 --racetrack-set mu60_closed --start-pose 0 0 0 \
  --output-dir Data/research/continuous_laps/my_session
python -B -m f1tenth_benchmarks.research.run_continuous_pp \
  --verify-only Data/research/continuous_laps/my_session
```

my_session 必須尚不存在。輸出包括 metadata、samples、圈事件與摘要；
`--save-scans` 可另記錄 LiDAR。這個入口仍使用既有 Pure Pursuit 與折線圈數追蹤，
沒有改為 Frenet 控制。圈追蹤的 s 與新 Frenet s 不可直接混用。

## 中線行駛與候選 session

使用既有模擬 Python 環境，從 repository 根目錄執行；NEW_SESSION 與 NEW_VERIFICATION.json 都須不存在：

```bash
python -B -m f1tenth_benchmarks.research.run_centerline_candidates \
  --output-dir Data/research/overtaking/NEW_SESSION --map my_map --laps 1 --plan-period 0.1 --lookahead 3 \
  --query-mode fast_with_fallback --generation-mode direct_then_repair
python -B -m f1tenth_benchmarks.research.overtaking.verify_session \
  Data/research/overtaking/NEW_SESSION --report NEW_VERIFICATION.json
```

車以固定 1.2 m/s 跟隨 Frenet CSV 中線；PP 前視未指定時按地圖解析：**my_map 為 0.3 m，esp 為 0.6 m**。可用 `--pp-lookahead` 明確覆寫；實際值保存於 session。候選每個 0.1 秒控制步更新，標準候選前視為 3 m，與 PP 前視分開。--map 可選 my_map／esp，主 CLI 地圖預設 esp。查詢與生成預設仍是 exact／optimized；獨立 1 mm 實驗的三號現在是**整段 d=0 的參考中線**，不再強制從實際車位／方向生成 Hermite 接回段，偏差由追蹤控制器處理。三號須獨立通過道路域與曲率檢查，且來源車況在設定的追蹤範圍內，才標記為可用保底；其餘四條仍保留實際起點契約。原 V5.2 入口與舊 `centerline_return` 實驗策略仍保留作對照。同步求解期間模擬時間暫停。session 保存實際車況、控制、五條候選（含失敗原因）、byte 索引及摘要；跨線完成時另保存終止規劃事件。候選只供診斷，不接管控制，不宣稱車身可安全通行。

本入口使用共用 core 控制與進度，不需 LMPC／osqp。驗證器做獨立物理重播、控制重算與來源／時間核對；瀏覽器時間步播放如下。異常 session 可加 `--allow-incomplete` 核對可完整重播的資料；runtime 中斷若尚有 action 未產生下一狀態，仍須先處理不完整區間。

固定定位資料重用可選 `--query-mode fixed_grid_lookup`，搭配 `--generation-mode direct_then_repair`；候選取樣仍為原 2.5 cm。此為已完成兩圈與原密度核驗的研究模式，尚未達 10 Hz，詳見[時間報告](CHANGELOG.md)。

### 播放已保存的 session

```bash
python -B -m f1tenth_benchmarks.research.overtaking.playback \
  --session Data/research/overtaking/COMPLETED_SESSION --port 8766
```

在本機開啟 http://127.0.0.1:8766/。支援播放／暫停、前後單步、相對時間／步號跳轉、拖曳時間軸、0.25×～4×、縮放／平移與圖層切換。只讀取原始記錄，不重新生成候選；失敗事件也照時刻呈現，回到以前不保留未來候選。

短預覽可先用 `--max-control-steps 3` 生成四筆車況的短預覽，再以同一播放入口檢查；畫面會明示未完成整圈。完整圈必須不設此上限。40 m 為歷史／壓力設定，主入口預設前視為 3 m。`--progress-session RUNNING_SESSION` 可另顯示背景生成的 progress.json，但不播放尚未完成的即時車況。

原始 session 的幾何版本、CSV、manifest、地圖來源必須與載入的 TrackData 一致。索引缺失或損壞可由 JSONL 重建，不寫回原始檔。程式只綁定 127.0.0.1；Docker 使用 host network 時也維持本機服務範圍。

## 生成 ESP 中線與邊界

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.build_map \
  --map-yaml maps/esp.yaml --anchor-csv maps/esp_centerline.csv \
  --centerline-method equidistant_periodic_v1 --centering-smoothing-m 0.3 \
  --output-root Data/research/geometry/my_build
```

使用 ESP 目前採用的距離場候選方法；CLI 預設仍為 nearest_wall_midpoint_v1。
新方法不保證適用所有地圖。anchor CSV 決定起點提示與正向，YAML 決定影像座標語意。
終端輸出 `output` 是本次 GEOMETRY_DIRECTORY，其中保存曲線、中線／邊界 CSV、
ROS waypoint CSV、manifest、驗證摘要與疊圖。相同版本目錄已存在會拒絕覆寫。

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.build_map \
  --verify-only GEOMETRY_DIRECTORY
```

## 查詢座標

可直接查詢既有 ESP 正式候選；若使用新生成版本，替換 geometry 路徑：

```python
from f1tenth_benchmarks.research.core.geometry import load_geometry

geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
frenet = geometry.to_frenet([0.0, 0.0], yaw=0.0)
cartesian = geometry.to_cartesian(10.0, 0.2)
limits = geometry.boundaries(10.0)
from f1tenth_benchmarks.research.core.geometry.section_ranges import query_section
section = query_section(geometry, 10.0)
print(frenet, cartesian, limits, section)
```

先看 valid，再使用座標；失敗原因在 reason。inspect=True 表示檢視候選，
目前不允許直接作規劃載入。圖形點選工具操作如下。

## 連續定位與 ROS2

一般 Python 定位可呼叫 tracker.update；ROS2 資料可走 update_tf，範例與
輸入要求見[介面定義](interfaces.md#ros2-定位介面)。部署端需自行取得 TF，
傳入同時基的目前時間；適配器不會自己訂閱 topic 或查詢 TF。

## 批次檢查與重播

只測幾何抽樣往返：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.check_frenet \
  --geometry GEOMETRY_DIRECTORY --map-yaml maps/esp.yaml \
  --output-dir Data/research/geometry/my_check
```

生成域樣本與有狀態重播報告：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.complete_stage_c \
  --geometry GEOMETRY_DIRECTORY --map-yaml maps/esp.yaml \
  --samples Data/research/continuous_laps/my_session/samples.jsonl \
  --output-dir Data/research/geometry/my_stage_c
```

兩個輸出目錄都須尚不存在。預設512截面、0.1 m橫向取樣，另加接縫與
問題區加密。`--points-json ANALYSIS_JSON` 可選擇重用舊診斷的固定 Cartesian 點。
沒有提供 samples 時仍可產生域報告，但不宣告完整 Stage C 通過。

C 輸出包含 `domain/validity.json`、`domain/manifest.json`、`report.json`、
`pose_contract.json`（英文檔名，內容是定位介面規格）與 `manifest.json`。
原幾何資料不變；執行時可 `load_domain('C_DIRECTORY/domain', geometry)`，
再用 `query(s, d)` 作精確檢查。批次報告不會自動批准全域規劃使用。

## Frenet 地圖檢視器

完整啟動與操作見[檢視器指南](viewer.md)。六圖既有候選可直接使用：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.viewer \
  --catalog maps/frenet/inspector_catalog.json --port 8765
```

catalog 路徑相對於該 JSON 檔目錄，各項指定 label、geometry、map_yaml、ranges。
若使用自己生成的版本：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.viewer \
  --geometry GEOMETRY_DIRECTORY --map-yaml maps/esp.yaml --port 8765
```

可加 `--ranges SECTION_DIRECTORY`、`--report D_REPORT_DIRECTORY` 或
`--diagnostics DIAGNOSTICS_DIRECTORY` 載入同版本的範圍／密集報告／診斷。
各資料須通過來源與版本校驗；正式 catalog 不依賴本機歷史 D 報告。

## 密集檢查與診斷

核對預先幾何並彙整已有密集報告的問題（輸出目錄必須尚不存在）：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.complete_stage_d \
  --geometry GEOMETRY_DIRECTORY --map-yaml maps/esp.yaml \
  --report D_REPORT_DIRECTORY --output-dir DIAGNOSTICS_DIRECTORY
```

產出 diagnostics.json 與 manifest.json。單圖 viewer 加 `--diagnostics DIAGNOSTICS_DIRECTORY`，
或填入 catalog 的 diagnostics。診斷只描述採樣結果，完整插值與規劃批准需另行驗證。

重新生成 D 第一批檢查，指定尚不存在的目錄：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.validate_stage_d \
  --geometry GEOMETRY_DIRECTORY --map-yaml maps/esp.yaml \
  --step-m 0.05 --max-depth 4 --output-dir NEW_D_REPORT_DIRECTORY
```

report／manifest 保存數值結果、拒絕點、邊界插值抽查、來源與程式雜湊。
保存的 ESP 密集報告 numeric_checks_passed=true、stage_d_complete=false；後者代表當時完整連續域／插值條件未批准。後續 D 診斷驗收已完成，以 acceptance.json 的 stage_d_diagnostics_complete=true 記錄；六圖本次轉換驗收也已完成。planning_allowed/domain_verified 均維持 false。

## 截面範圍匯出

在既有 Python／Docker 環境使用下列入口（輸出目錄必須尚不存在）：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.section_ranges \
  --geometry GEOMETRY_DIRECTORY --map-yaml maps/esp.yaml \
  --s-step-m 0.2 --d-step-m 0.025 --output-dir NEW_SECTION_DIRECTORY
```

輸出 `centerline.csv`、`boundaries.csv`（沿用該幾何版本）、
`section_ranges.json`（兩種範圍、限制及樣本）、`report.json`（驗證摘要）、
`manifest.json`（來源／程式／檔案雜湊）。JSON 保留多個區間與原因，避免 CSV
單一上下限掩蓋中間的拒絕區。執行時任意 s 由同一 API 重新精確查詢道路並採樣。

## 開放走廊中線

閉合 AUT／GBR／MCO 使用各圖既有 anchor 與相同週期提取方法。CornerHall
沒有既有中線，依使用者選定的下端向上再向左，使用開放提取入口：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.build_open_map \
  --map-yaml maps/CornerHall.yaml --start-hint 5.336 -16.525 \
  --output-root NEW_OPEN_GEOMETRY_ROOT
```

start-hint 必須位於 free 區且接近終端；它選擇骨架起端，並記錄主走廊路徑選擇。
只在種子提取時抑制≤0.02 m²的小孔洞；原 occupancy 不變，所有中線樣本與
左右邊界仍對原圖核對。多岔路的其他支線不納入這條參考線；必須檢視方向與覆蓋。
接著對輸出的 geometry 使用上方 section_ranges 指令，保存開放區間兩端。
正式內建圖統一位於 `maps/frenet/<地圖>/geometry/` 與 `ranges/`；
不在正式路徑加入批次號或 geometry_id 層級，CSV 只保存在 geometry。完整樣本保存在 ranges/section_ranges.json.gz，
檢視器直接解壓讀取，manifest 保留原數值版本與無損包裝的來源紀錄。


## 正式地圖資料與研究紀錄

正式檔案結構見 [maps/frenet/README.md](../../maps/frenet/README.md)。六張圖使用相同
geometry／ranges 層級，CSV 可直接開啟；gzip JSON 的 Python 讀法如下：

```python
import gzip, json
with gzip.open("maps/frenet/aut/ranges/section_ranges.json.gz", "rt", encoding="utf-8") as f:
    sections = json.load(f)
```

研究輸出先存到本機 `Data/research/geometry/`。驗證後可建立新的整理目錄：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.publish_tracks \
  --catalog VALIDATED_CATALOG_JSON --output-root NEW_PUBLICATION_DIRECTORY
```

工具核對來源、幾何、範圍、CSV 一致性及驗證結果，保留全部數值，無損壓縮 JSON，
建立相對路徑 catalog；目標存在時拒絕覆蓋。檢查候選差異後再更新正式版本。
整理前研究紀錄位於 `Data/research/geometry/archive/2026-10-07_before_publication/`；
原報告不改寫，`archive/location_map.json` 記錄搬移，`archive/inspector_catalog.json`
可另外開啟歷史版本。Git 不包含 archive。


## 實際 PGM＋YAML 轉換

本次使用者提供的測試圖已保存為 maps/my_map.pgm。maps/my_map.original.yaml
保留原始設定；工作 maps/my_map.yaml依使用者確認將free_thresh從0.25改為0.196，
讓灰色205為未知區。來源hash、起點與結果摘要見 maps/my_map.import.json。

沒有既有中線的閉合賽道可指定方向，於較平直區段自動選擇s=0：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.build_external_map \
  --map-yaml maps/my_map.yaml --original-yaml maps/my_map.original.yaml \
  --direction counterclockwise --output-root NEW_GEOMETRY_ROOT
```

自動選取要求只有一個支援的封閉走廊；多個候選需加 --start-hint X Y，開放走廊
使用 build_open_map。此入口不自動改門檻，先按 YAML 分類 occupancy，對原圖
計算雙側等距種子，再用相同平滑／精確邊界與數值驗證方法。
接著使用 section_ranges 匯出與 publish_tracks 整理。my_map目前已加入正式catalog，
啟動現有檢視器後在右上選取「my_map｜實際 PGM」，即可查看中線、邊界與s/d。


## my_map 有預算的 B-spline 修正實驗

保留三號參考中線，只對失敗的二、四號選擇性修正；不改既有預設入口。
在既有模擬器 Python 環境執行，輸出目錄必須尚不存在：

```bash
python -B -m f1tenth_benchmarks.research.run_mm_bspline_comparison \
  --domain-dir Data/research/overtaking/mm_domain_width27_length51_20261030_001/domain \
  --output-dir Data/research/overtaking/NEW_BSPLINE_COMPARISON \
  --budgets-ms 0 20
```

單車回放保留 20 ms 與 40 ms 修 C1／2／4／5 對照。一般多預算比較的第一組必須為 0，作為相同控制條件的基準。單組測試可只指定 `--budgets-ms 20`。可額外加入 40／80 ms 作比較，不會自動變更目前選擇。預算包含基本生成，不含 session gate
及控制；截止為協作式檢查與丟棄過時結果，不是底層硬即時中斷。
此單車對照不讓候選接管控制；安全域只檢查中心路徑。
目前新車體結果見[27×51 cm 重驗](CHANGELOG.md)與[現況](CHANGELOG.md)；[首次 B-spline 預算實驗](CHANGELOG.md)為舊車寬對照。

目前車體參數為寬 27 cm、長 51 cm、軸距 33 cm、轉角上限 29.5°；舊 20 cm 車寬的安全域不可混用。新參數比較與完成進度見 [重驗報告](CHANGELOG.md)。


## my_map／ESP 實驗回放（8768）

本機 8768 預設開啟 [ESP 完整測試](http://127.0.0.1:8768/#esp_complete_test)。兩圖的單車 40 ms 對照和 20 ms GIF 都在歷史基準。

歷史 my_map 20 ms GIF 位於 `/data/f1tenth/sites/my-map-lab-20261029/dist/my_map_20ms.gif`，也可由網頁下載。網頁由 `f1tenth-my-map-lab` 容器持續提供；匯出器為 `research.export_mm_dashboard`，播放及來源一致性檢查位於 `/data/f1tenth/sites/my-map-lab-20261029/check.mjs`。


ESP 單組跨圖驗證可沿用同一入口與規劃器，明確指定地圖並建立新的安全域：

```bash
python -B -m f1tenth_benchmarks.research.run_mm_bspline_comparison \
  --map esp --build-domain --budgets-ms 20 \
  --domain-dir Data/research/overtaking/NEW_ESP/domain \
  --output-dir Data/research/overtaking/NEW_ESP/validation
```

安全域目錄及驗證輸出須尚不存在；ESP 自動 PP 前視為 0.6 m，規劃距離仍為 3 m。
單組不宣稱完成跨方法同車況對照；所有接受曲線均另做原始密度幾何稽核，代表步另做 1 mm 複查，結果寫入同批 report.json。


ESP 可另指定 `--repair-candidates 1 2 4 5 --budgets-ms 40`，在共用預算內修正所有非參考候選；三號不允許進修正器。`--compare-outer --budgets-ms 40` 則先跑只修 2／4，再跑 1／2／4／5 兩組完整圈，最後獨立複查。兩組共用同一離線安全域，先中間、後外側；不是每條曲線各自有 40 ms。


## 最新快照重跑與資料依賴

Git 版本保存程式、參數、測試、代表圖，以及 `docs/project/reports/latest_experiments/` 的精簡結果與來源 SHA256。原始逐步 session、1 mm domain.npz 與大型歷史圈留在本機 Data，不會隨 clone 自動取得。已有地圖幾何可從 Git 取得；新的 40 ms 完整圈可建立自己的安全域與輸出：

```bash
python -B -m f1tenth_benchmarks.research.run_mm_bspline_comparison \
  --map my_map --build-domain --compare-outer --budgets-ms 40 \
  --domain-dir Data/research/overtaking/NEW_MY_MAP/domain \
  --output-dir Data/research/overtaking/NEW_MY_MAP/validation
```

此命令比較只修 C2／C4 與修 C1／C2／C4／C5。將 map 改成 esp 並使用新目錄可重跑 ESP 單車對照。

CPU 與 warm start 的重跑工具使用已保留的原始錄製車況：

```bash
python -B -m f1tenth_benchmarks.research.run_cpu_baseline \
  --output Data/research/overtaking/NEW_CPU/report.json
python -B -m f1tenth_benchmarks.research.run_aligned_warm_start \
  --output Data/research/overtaking/NEW_WARM_START
```

兩個工具目前固定讀取 `cpu_baseline`／`warm start` 程式中列出的 2026-10-30 原始 session 和對應安全域；只 clone Git 後不能直接重跑，需取得相同原始資料，或修改研究入口的來源路徑使其指向新錄製資料，並記錄新的輸入 hash。原因分析工具也固定讀取既有 warm start 批次，見 CHANGELOG 第 11～13 節。它們都是固定車況的規劃量測，沒有重新跑車、ROS2 節點或 Jetson。

8768 使用獨立 Git repository `/data/f1tenth/sites/my-map-lab-20261029`，在該目錄執行：

```bash
node check.mjs --assets-only
python3 -m http.server 8768 --bind 127.0.0.1 --directory dist
```

若既有 8768 容器正在服務，不要另啟同埠。網頁保存自足的匯出資料，可離線回放；`node check.mjs` 不加旗標會進一步核對本機大型原始資料 SHA256，需要原始 Data 仍在。對應網頁 commit 記於最新結果 manifest，歷史 20 ms GIF 明確標示用途。
## 單車候選路線三圓檢查（2026-11-05）

`run_centerline_candidates` 與經過 `session.make_plan` 的 1 mm／40 ms
研究生成流程，現在預設在候選發布前執行三圓車體檢查，包含 C3 中線及
B-spline 修正後候選。原中心路徑域與曲率預篩仍保留。直接呼叫底層生成器
不會執行此最終發布關卡。

車長、車寬、牆面餘量預設讀取 `params/simulator_params.yaml` 的
`vehicle_length=0.51`、`vehicle_width=0.27`、`vehicle_boundary_clearance=0.05`。
圓心為車體縱向 −L/3、0、L/3，半徑 `sqrt((L/6)^2+(W/2)^2)`；
檢查半徑再加牆面餘量。沿原始解析曲線每 0.01 m 的參考 s 取樣位置與切線朝向，
包括終點；不拿顯示點或只有起點的記錄當完整路線。
使用規劃器原始 trinary map，未知格與圖外均視為障礙，檢查整個像素方格。
當前實際車體位置也須通過，C3 的參考路線不能繞過此檢查。

主 CLI 可調 `--footprint-clearance 0.05 --footprint-step 0.01`。
程式設定為 `SessionConfig.footprint_*`，隨 `metadata.json` 保存。
`plans.jsonl` 的 `footprint_validation` 記錄幾何設定，各候選的
`footprint_check` 記錄拒絕原因、首個不合格姿態；`footprint_sampled_valid`
表示取樣檢查通過，失敗會更新 `sampled_valid`、有效 ID 及零候選狀態。
`footprint_check_enabled=False` 僅供明確重現舊版／無地圖單元測試。

這是候選路線的取樣車體檢查，尚非連續曲線或追蹤誤差的安全證明，
因此 `footprint_verified`／`planning_allowed` 保持 false。
原單車物理碰撞判定、PP 中線控制及 8768 舊回放不在本次變更範圍。
三圓檢查時間包含在 `make_plan.wall_time_s`，不在生成器的 40 ms 修正預算內。

## 三圓車體與雙車基本運行（2026-11-05）

在專案根目錄、既有 `f1tenth-sim:full` 環境執行：

```bash
python -m f1tenth_benchmarks.research.run_two_car \
  --map my_map --seconds 12 --gap 1.5 \
  --ego-speed 1.2 --lead-speed 1.2 \
  --wall-clearance 0.05 --pair-clearance 0.05 \
  --output Data/research/overtaking/two_car_new_run
```

輸出目錄必須尚不存在；輸出 `report.json`、`frames.json`、`preview.png`。
兩車各自使用既有動力學與中線 PP；前車速度可設 0 作為靜止車。
此獨立入口採三圓包覆 51×27 cm 車體：縱向圓心 −17／0／17 cm，
半徑 `sqrt((L/6)^2+(W/2)^2)` 約 15.95 cm。車體姿態暫沿用幾何中心假設。
牆面以原圖佔用像素方格判定，圖外視為障礙；沿用模擬器像素 <=128 的定義。
牆面餘量加在圓半徑上，車間餘量只在兩車半徑和上加一次。
觸發包覆接觸或餘量不足即停止，事件不代表精確矩形已碰撞。
物理步長 10 ms、控制步長 100 ms；物理步間依位置線性／最短方向角插值，
以膨脹中點圓保守包覆插值掃掠，非連續動力學的碰撞證明。
LiDAR 包含對方未加餘量的三圓，關閉量測雜訊以利基本核對。
此入口尚不包含候選接管、超車決策或碰撞後力學反應。

## my_map 無光達雙車候選跟隨（Phase 1）

在專案根目錄執行。以下以兩車等速驗證持續跟隨，輸出目錄須尚不存在。

```bash
python -m f1tenth_benchmarks.research.run_two_car_candidates \
  --map my_map --lead-speed 1.2 --seconds 12 \
  --output Data/research/overtaking/two_car_candidates_equal
```

入口輸出 `report.json`、`frames.json` 和 `preview.png`。省略
`--domain-dir` 時先核對既有安全域。若找不到相容資料，會在輸出目錄下保存
重建的 `domain/`，後續可用 `--domain-dir` 明確重用。用不同輸出目錄和
`--lead-speed 0.9` 可驗證後車追近時的車間餘量終止。用
`--preferred-candidate 2` 可測試側向跟隨。Phase 1 依靜態驗證與候選偏好
選路徑，車間餘量由同步物理模擬判定。

## Phase 2 前車預測、減速跟車與超車

加上 `--phase 2` 使用時空候選篩選與規則式排名。若已有通過來源核對的安全域，
可用 `--domain-dir` 重用。以下由原雙車預設的後車 1.2／前車 0.9 m/s、
起始間距 2 m 重跑 my_map，輸出目錄須尚不存在。

```bash
python -m f1tenth_benchmarks.research.run_two_car_candidates \
  --phase 2 --map my_map --seconds 12 \
  --output Data/research/overtaking/two_car_phase2_my_map_new
```

ESP Phase 2 基準使用 `--phase 2 --map esp --seconds 18 --ego-speed 1.35 --lead-speed 0.85 --gap 2`。回放見 [my_map 跟車](http://127.0.0.1:8768/#two_car_phase2_my_map)和 [ESP 基準](http://127.0.0.1:8768/#two_car_phase2_esp)；規則見[雙車決策](interfaces.md#雙車候選與超車決策)。

### ESP 原版轉向節奏對照

`--original-esp-control` 僅適用於 ESP Phase 2，使用速度連動前視與 40 ms 轉向。

```bash
python -m f1tenth_benchmarks.research.run_two_car_candidates \
  --phase 2 --map esp --original-esp-control --rolling-start \
  --ego-speed 4.0 --lead-speed 3.2 --ego-start-s 34.5 --gap 1.5 \
  --path-lookahead-m 4.1 --seconds 6 \
  --output Data/research/overtaking/esp_original_control_bend_new
```

要重現受阻先跟車，將後車速度、起始弧長、間距、候選距離和時長改為
`--ego-speed 3.6 --ego-start-s 193 --gap 0.84 --path-lookahead-m 4.5 --seconds 8`，
並使用新的輸出目錄。

### ESP 三圈驗收與實測彎速

前次三圈對照使用中線曲率限速與[兩段實測彎速](CHANGELOG.md#esp-雙車三圈跟車判定與彎道限速2026-11-12)。

```bash
python -m f1tenth_benchmarks.research.run_two_car_candidates \
  --phase 2 --map esp --original-esp-control --curve-speed-control --measured-bend-caps \
  --rolling-start --laps 3 --seconds 280 --ego-start-s 34.5 --gap 1.5 \
  --ego-speed 4 --lead-speed 3.2 --path-lookahead-m 4.1 \
  --domain-dir Data/research/overtaking/two_car_phase2_esp_domain_20261112 \
  --output Data/research/overtaking/esp_three_laps_new
```

### ESP 完整測試：起點出發、最佳化賽道線與套圈

兩車從起跑區靜止出發，Car 1 目標 2.5 m/s；Car 0 的最佳化線上限 8 m/s。切換與計數見[雙車決策](interfaces.md#雙車候選與超車決策)。若沒有下列本機安全域，省略 `--domain-dir` 整行：程式會沿用相容的 ESP 安全域，找不到時重建至本次輸出目錄的 `domain/`。輸出目錄須尚不存在。

```bash
python -m f1tenth_benchmarks.research.run_two_car_candidates \
  --phase 2 --map esp --original-esp-control --curve-speed-control --measured-bend-caps \
  --laps 3 --seconds 320 --ego-start-s 0 --gap 1.5 \
  --ego-speed 4 --lead-speed 2.5 --path-lookahead-m 4.1 \
  --raceline-path Data/racelines/mu60_closed/esp_raceline.csv --raceline-speed 8 \
  --domain-dir Data/research/overtaking/two_car_phase2_esp_domain_20261112 \
  --output Data/research/overtaking/esp_complete_continuous_8_new
```

本機 8768 的 `#esp_complete_test` 回放提供最佳化線圖層、峰值車速與套圈切換跳轉；
地圖鏡頭可切換全圖、雙車、Car 0 或 Car 1。
