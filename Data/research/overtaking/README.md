# 規劃實驗原始資料

這裡保存原始錄製、安全域與機器驗證資料；結果集中在[實驗紀錄](../../../docs/project/CHANGELOG.md)，重跑方式見[操作指南](../../../docs/project/usage.md)。新實驗用新目錄，原始錄製預設不納入 Git。

| 目前常用資料 | 目錄 |
|---|---|
| ESP 8 m/s 完整測試 | `esp_complete_continuous_8_wait_20261112/` |
| 網站 my_map 減速跟車 | `two_car_phase2_my_map_final_20261112/` |
| my_map 側向退回中線（未上網站） | `two_car_phase2_my_map_catchup/` |
| ESP 雙車完整測試安全域 | `two_car_phase2_esp_domain_20261112/` |

舊的兩圖單車 40 ms 對照是 `my_map_mm40_outer_20261030_001/`、`esp_mm40_outer_20261030_001/`；20 ms 基準、CPU 與 warm start 的來源雜湊見[歷史結果清單](../../../docs/project/reports/latest_experiments/manifest.json)。單車對照由 PP 追蹤中線；雙車則可由候選接管。安全域須核對地圖、幾何與餘量，不可跨圖混用。

歷史探測歸檔見[舊實驗 ZIP](../../../docs/project/archive/experiments_through_20261030.zip)及本目錄的 `cleanup_20261030.json`。網站的衍生回放另存在 `/data/f1tenth/sites/my-map-lab-20261029/dist`。
