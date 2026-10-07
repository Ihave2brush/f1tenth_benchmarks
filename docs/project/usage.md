# 操作指南

所有指令從 repository 根目錄執行。原版 benchmark 入口見根目錄 README；
以下只說明新增研究功能。資料產物通常不在 Git，下載專案不代表已有歷史 session。

完成狀態與保存的測試結果見[完成總結](status.md)。以下指令用於重現操作，不代表本次文件整理有重新執行。

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
