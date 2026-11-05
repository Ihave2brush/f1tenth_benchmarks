# 規劃實驗原始資料

修改過程與結果只維護 `docs/project/CHANGELOG.md`。本目錄是原始 session、安全域與機器驗證資料，沒有另寫人工進度紀錄。

最新展示使用 `my_map_mm40_outer_20261030_001/validation/budget_40ms_c1245` 與 `esp_mm40_outer_20261030_001/validation/budget_40ms_c1245`；歷史 20 ms 基準為 `mm_bspline_width27_length51_20261030_001/budget_20ms`。車體 27×51 cm、軸距 33 cm、轉角 29.5°、PP 前視 0.3 m、規劃前視 3 m。安全域在 `mm_domain_width27_length51_20261030_001/domain`，中心距預篩為 18.5 cm。原始 states/actions/plans/metadata 不可覆寫；每次新實驗使用新目錄。

歷史完整圈、各組對照與舊安全域仍保留供重播及核驗。`mm_bspline_probe_20261030_001` 探測輸出已歸檔；目前網站所需匯出檔保留在 `/data/f1tenth/sites/my-map-lab-20261029/dist`，重複的 web_export 已清理。清理細項見 `cleanup_20261030.json`，歷史原文與非現行圖形位於 `docs/project/archive/experiments_through_20261030.zip`。

原始資料預設不納入 Git，程式／測試必要 JSON 輸入與精簡結果在 `docs/project/reports/` 保留。PP 目前只追蹤參考中線；候選未接管控制，無碰撞紀錄不等於完整車體 5 cm 安全距證明。

CPU 比較為 `cpu_baseline_20261105/report.json`；位移 warm start 與原因分析在 `aligned_warm_start_20261105_001/`，預設未採用對齊版。Git 中的精簡結果副本與來源 SHA256 保存於 `docs/project/reports/latest_experiments/manifest.json`；原始路徑不變。ESP 安全域沿用 `esp_mm_20ms_20261030_001/domain`，不得混用 my_map 安全域。
