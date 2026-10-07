# 相較原版的功能差異

原版方法與入口見 [repository README](../../README.md)。新增研究功能位於 `f1tenth_benchmarks/research/`，完成狀態與數值證據統一見[完成總結](status.md)。

| 新增功能 | 用途 | 主要入口（相對 research/） |
|---|---|---|
| Continuous 多圈模擬 | 一次初始化，跨圈保留物理狀態與轉向 buffer | `continuous_sim.py`、`run_continuous_pp.py` |
| 閉合速度配置 | 納入末段到首段的速度與煞車約束 | `closed_velocity_profile.py` |
| 圈事件追蹤 | 區分 partial／full、起步／flying；拒絕異常跳躍 | `lap_tracking.py` |
| 紀錄與重播 | 保存 post-step 狀態、動作、時間、圈事件與版本 | `continuous_logging.py`、`run_continuous_pp.py` |
| 地圖幾何生成 | 從影像／YAML 建立閉合賽道或開放走廊參考線與邊界 | `core/geometry/build_map.py`、`build_open_map.py`、`build_external_map.py` |
| Frenet 雙向轉換 | x/y↔s/d、投影候選、occupancy 與曲率餘裕檢查 | `core/geometry/frenet.py` |
| 連續定位追蹤 | 利用歷史、時間與航向追蹤分支及跨接縫進度 | `core/geometry/tracker.py` |
| 截面與域查詢 | 道路 d 範圍、採樣 Frenet 區間、限制原因及精確查詢 | `core/geometry/section_ranges.py`、`domain.py` |
| 多圖檢視與診斷 | 點選、s/d／截面查詢、候選及拒絕原因 | `core/geometry/viewer.py`、`viewer.html` |
| TF 資料適配 | 檢查 map→base_link 相容資料的框架、時間、版本與偏移 | `FrenetTracker.update_tf()` |
| 共用研究入口 | simulator／lap tracker／logger 轉匯出既有實作 | `core/continuous_sim.py`、`core/lap_tracking.py`、`core/logging.py` |

Continuous 沿用原動力學、LiDAR、碰撞模型與 Pure Pursuit；新增的是模擬生命週期、圈事件與紀錄。Frenet 是另一組共用幾何能力，目前尚未接入該控制迴路。

超車與 LMPC 只有預留目錄。TF 適配器尚未構成 ROS2 訂閱／TF buffer／publisher 節點；路線規劃、車身／動態驗證與部署需另行完成。
