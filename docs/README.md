# 專案文件

雙車從這裡開始：[現行決策](project/interfaces.md#雙車候選與超車決策) · [ESP 完整測試重跑](project/usage.md#esp-完整測試起點出發最佳化賽道線與套圈) · [實驗結果](project/CHANGELOG.md#esp-完整測試8-ms急彎先跟車持續生成候選2026-11-12)。

## 必要技術指南

| 文件 | 用途 |
|---|---|
| [修改過程與結果](project/CHANGELOG.md) | 唯一持續維護的修改／實驗紀錄 |
| [操作指南](project/usage.md) | 安裝、測試、規劃、資料生成與播放命令 |
| [介面與座標](project/interfaces.md) | 來源、座標、候選及驗證契約 |
| [功能入口](project/features.md) | 程式模組與用途對照 |
| [地圖檢視器](project/viewer.md) | 8765 的六圖查詢方式 |
| [Frenet 技術設計](project/frenet_runtime_validation_plan.md) | 精確查詢、快取與有效性設計 |
| [中線與播放設計](project/centerline_candidates_playback_plan.md) | 記錄／播放架構與契約 |

本機頁面：[8765 地圖查詢](http://127.0.0.1:8765/)、[8766 session 播放](http://127.0.0.1:8766/)、[8768 ESP 完整測試與歷史回放](http://127.0.0.1:8768/#esp_complete_test)。

## 歷史實驗與驗收報告

[雙車 Phase 1 原始規劃](project/two_car_candidate_overtaking_plan.md)保留最初的介面設計與驗收項目。

原先分散的 28 份報告／進度／工作計畫、9 組非現行圖形及探測輸出，已核對後放入單一[歸檔 ZIP](project/archive/experiments_through_20261030.zip)。原名、各檔 SHA256 與安全取回方式見[統一紀錄](project/CHANGELOG.md)及 ZIP 中的 MANIFEST.json。

`project/reports/` 只保留程式／測試輸入、機器可讀結果 JSON 與目前新車體代表圖；原始完整圈與安全域仍保留在本機 `Data/research/`，目前 8768 資料與 GIF 也保持可用。原始資料不因文件整理而改寫。
