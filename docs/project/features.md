# 相較原版的功能差異

原版方法與入口見 [repository README](../../README.md)。新增研究功能位於 `f1tenth_benchmarks/research/`，完成狀態與數值證據統一見[完成總結](CHANGELOG.md)。

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
| 中線行車與五候選 | Frenet CSV 中線 PP、定位／進度、五 ID 候選與 session | `run_centerline_candidates.py`、`overtaking/session.py` |
| 記錄播放與重驗 | 依時間／步號讀取保存資料，物理／控制／來源／索引核驗 | `overtaking/playback.py`、`verify_session.py` |
| 查詢加速實驗 | 單次共用、精確 s 道路表、跨次快取與批次弧長反解 | `overtaking/geometry_queries.py`、`road_table.py`、`fixed_arc_lookup.py` |
| 生成策略實驗 | 原最佳化、直接生成後修正與寬度比例生成對照 | `overtaking/robust_adaptive_paths.py`、`width_fraction_paths.py` |
| 1 mm 安全域 | 離線整格距離判定、保守內縮包絡、上下界及位元表／版本核對 | `overtaking/millimetre_domain.py` |
| 三號參考契約 | 純 d=0 中線，與匹配實際起點的軌跡分開核對 | `overtaking/reference_path.py` |
| 有預算 B-spline 修正 | 只修正失敗二／四號，保留快速直接曲線與每次驗證 | `overtaking/mm_bspline_repair.py`、`run_mm_bspline_comparison.py` |
| 雙賽道 40 ms 網頁／歷史 GIF | 兩圖完整圈回放、逐步診斷、離線預計算；GIF 保留歷史 20 ms | `export_mm_dashboard.py`；本機 8768 |
| 共用研究入口 | simulator／lap tracker／logger 轉匯出既有實作 | `core/continuous_sim.py`、`core/lap_tracking.py`、`core/logging.py` |

Continuous 沿用原動力學、LiDAR、碰撞模型與 Pure Pursuit；尺寸／軸距／轉向限制採目前已確認參數。原 raceline 多圈入口與 Frenet 中線候選入口分開；後者已接上同版本 CSV、Frenet 定位／進度、五候選記錄及瀏覽器時間步播放。入口支援 ESP／MY map，最新研究設定為 MY map／3 m，主 CLI 地圖預設仍 esp。

最新展示為 my_map／ESP、1 mm 安全域＋直接生成＋40 ms 共用預算修正一／二／四／五號；完整週期 P95 分別 30.12／46.87 ms，最大 38.52／48.93 ms，僅代表此批量測落在 100 ms 預算內，硬即時尚未驗收。主入口仍為 exact／optimized，獨立研究入口與契約見[介面規格](interfaces.md#候選生成與-session-契約)。fixed_grid_lookup 的兩圈與 10/29 超時結果保留為歷史對照，不能當作最新方案的時間。歷史 ESP 40 m 全量手動播放最終驗收仍待完成。數值與證據集中於[完成總結](CHANGELOG.md)。

候選不接管控制；超車決策、完整車身／動態、ROS2 節點及實車部署尚未驗收。LMPC 維持獨立分支；TF 適配器不是完整 ROS2 訂閱／TF buffer／publisher 節點。
