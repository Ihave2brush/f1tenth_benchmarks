# F1TENTH Benchmarks 與模擬研究

以原版 F1TENTH Autonomous Racing Benchmarks 為基礎，新增 **Continuous 連續圈模擬**、**Frenet 地圖幾何／座標查詢**與**中線行車／五候選記錄及播放**。沿用原版動力學、LiDAR、碰撞檢查及 benchmark 入口，車輛尺寸／軸距／轉向限制按目前確認參數設定。

- [修改過程與結果](docs/project/CHANGELOG.md)：唯一持續維護的修改／實驗紀錄，包含採用方案、成果、限制與清理結果。
- [操作指南](docs/project/usage.md)：安裝、測試、連續圈與地圖生成。
- [Frenet 檢視器](docs/project/viewer.md)：六圖切換、點選與截面範圍查詢。
- [介面與座標定義](docs/project/interfaces.md)：程式接線與有效性規則。
- [文件索引與維護原則](docs/README.md)。

## 目前完成範圍

| 功能 | 已完成 | 驗證範圍 |
|---|---|---|
| Continuous | 閉合速度配置、一次初始化的多圈模擬、圈事件、紀錄與重播 | ESP／GlobalPurePursuit；已保存三圈與五圈驗收 |
| Frenet | 中線／邊界、雙向座標轉換、定位追蹤、截面範圍與檢視器 | ESP、AUT、GBR、MCO、CornerHall 與實際 PGM `my_map` |
| 中線與五候選 | Frenet CSV 中線 PP、定位／進度、五 ID 候選 session 與時間步播放 | ESP／MY map；新車體 40 ms 全候選組在兩圖完成一圈及獨立核驗，候選不接管 |
| 查詢與生成加速 | 1 mm 離線安全域、公式生成、外側推論、可選一／二／四／五號限時 B-spline 修正 | 最新 40 ms 共用預算；my_map 完整週期 P95 30.12 ms、ESP 46.87 ms；硬即時尚未驗收 |
| 後續研究 | 修正與區段重用、候選接管、超車決策、ROS2 與實車 | LMPC 維持獨立研究分支；各項另行驗收 |

原 Continuous 入口保留 raceline 與折線圈追蹤；中線候選入口已使用 Frenet CSV 控制路徑與 Frenet 定位。候選仍只供記錄與診斷，`planning_allowed=false`；同步規劃期間模擬時間暫停，有限採樣與多圈無碰撞不等於即時或車身安全驗收。

最新已測試方案為 **my_map／ESP、1 mm 安全域＋直接生成＋40 ms 共用預算修正 C1／2／4／5**，三號為 d=0 中線。車寬 27 cm、長 51 cm、軸距 33 cm、最大前輪轉角 ±29.5°；規劃前方 3 m、速度 1.2 m/s、控制／規劃週期 100 ms。PP 前視 my_map 0.3 m、ESP 0.6 m；中心預篩 18.5 cm，尚非完整車體掃掠安全證明。

歷史 20 ms 僅修二／四號仍保留作對照。位移對齊 warm start 已完成三輪配對，未顯示整體加速，因此預設保留原係數 warm start。主 CLI 仍預設 esp／exact／optimized，研究方案須明確選用，見[操作指南](docs/project/usage.md#my_map-有預算的-b-spline-修正實驗)。40 ms 是合作式生成預算，不是每條各 40 ms，也不是完整週期硬上限。

網頁入口：[8765 地圖幾何](http://127.0.0.1:8765/)、[8766 session 播放](http://127.0.0.1:8766/)、[8768 雙賽道最新 40 ms 實驗](http://127.0.0.1:8768/)。8768 在獨立 repository `/data/f1tenth/sites/my-map-lab-20261029`；本 repository 的[結果快照 manifest](docs/project/reports/latest_experiments/manifest.json)記錄對應網頁 commit 與證據來源。大型原始資料留在本機，Git 保存程式、測試、設定、精簡結果與代表圖。

## 開始使用

研究環境使用 Python 3.9。從本 repository 根目錄執行：

```bash
git submodule update --init --recursive
python -m pip install -r requirements-ci.txt
python -m pip install --no-deps ./trajectory_planning_helpers
python -m pip install --no-deps -e .
python -m pip check
pytest -p no:cacheprovider
```

helper 使用一般安裝。既有 `f1tenth-sim:full` 映像可用 `bash tests/run_research_tests.sh`；腳本需要 Docker 與依賴下載網路。完整操作與測試邊界見[操作指南](docs/project/usage.md)。

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.viewer \
  --catalog maps/frenet/inspector_catalog.json --port 8765
```

在瀏覽器開啟 [本機檢視器](http://127.0.0.1:8765)。正式候選資料見 [maps/frenet](maps/frenet/README.md)。

## 原版 benchmark

原版提供 classical、mapless、end-to-end 與 local-map racing：

| 類型 | 主要方法 |
|---|---|
| Classical | 粒子濾波定位、raceline 生成、Pure Pursuit、MPCC |
| Mapless | Follow the Gap |
| End-to-end | SAC／TD3 學習代理 |
| Local-map | LiDAR 局部地圖搭配 MPCC／Pure Pursuit |

原版執行腳本位於 `f1tenth_benchmarks/run_scripts/`，設定在 `params/`；[quickstart.ipynb](quickstart.ipynb)保留原版示例。[論文結果重現說明](f1tenth_benchmarks/benchmark_results/benchmark_results.md)屬上游 benchmark，不代表新增研究功能的驗收。MPCC／DRL 等完整依賴另見 `requirements.txt`，研究 CI 的精簡環境不涵蓋所有演算法。

## 授權與引用

沿用 [Apache 2.0 授權](LICENSE.md)，第三方 helper 的授權與文件保留於其目錄。使用原版研究成果時請引用：

```bibtex
@article{evans2024unifying,
  title={Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks},
  author={Benjamin David Evans and Raphael Trumpp and Marco Caccamo and Felix Jahncke and Johannes Betz and Hendrik Willem Jordaan and Herman Arnold Engelbrecht},
  journal={arXiv preprint arXiv:2402.18558},
  year={2024}
}
```
