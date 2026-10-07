# F1TENTH Benchmarks 與模擬研究

以原版 F1TENTH Autonomous Racing Benchmarks 為基礎，新增 **Continuous 連續圈模擬**與 **Frenet 地圖幾何／座標查詢**。原版動力學、LiDAR、碰撞檢查及 benchmark 入口持續沿用。

- [完成總結與測試紀錄](docs/project/status.md)：從基礎到 Continuous、Frenet 的成果與驗證範圍。
- [操作指南](docs/project/usage.md)：安裝、測試、連續圈與地圖生成。
- [Frenet 檢視器](docs/project/viewer.md)：六圖切換、點選與截面範圍查詢。
- [介面與座標定義](docs/project/interfaces.md)：程式接線與有效性規則。
- [文件索引與維護原則](docs/README.md)。

## 目前完成範圍

| 功能 | 已完成 | 驗證範圍 |
|---|---|---|
| Continuous | 閉合速度配置、一次初始化的多圈模擬、圈事件、紀錄與重播 | ESP／GlobalPurePursuit；已保存三圈與五圈驗收 |
| Frenet | 中線／邊界、雙向座標轉換、定位追蹤、截面範圍與檢視器 | ESP、AUT、GBR、MCO、CornerHall 與實際 PGM `my_map` |
| 後續研究 | 目錄與介面預留 | 路線規劃、超車、LMPC、ROS2 Frenet 節點與實車驗證尚未完成 |

Frenet 目前供檢視與精確查詢，`planning_allowed=false`；尚未接入 Continuous 的 Pure Pursuit 控制迴路。有限採樣通過不等於全道路連續域批准。

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
