# 專案文件

首次閱讀先看[完成總結](project/status.md)，了解基礎、Continuous 與 Frenet 如何銜接，以及哪些成果已有測試證據。

| 文件 | 回答的問題 |
|---|---|
| [完成總結與測試紀錄](project/status.md) | 完成了什麼？如何驗證？還缺什麼？ |
| [功能差異](project/features.md) | 相較原版新增哪些功能？ |
| [操作指南](project/usage.md) | 如何安裝、執行與產生資料？ |
| [介面與座標定義](project/interfaces.md) | 輸入輸出、座標與有效性怎麼接？ |
| [Frenet 檢視器](project/viewer.md) | 如何看六張圖、查詢與解讀結果？ |
| [Continuous 五圈驗收報告](project/reports/manual_run_001_report.md) | 行車結果、圖表、規劃對照與重播證據 |

## 維護與對外閱讀

上述文件、代表性案例圖與 [Frenet 資料說明](../maps/frenet/README.md)適合隨 repository 提供讀者閱讀。完成狀態與驗收數值只在完成總結集中維護；操作文件不追加每批測試流水帳。

本機 `docs/local/` 管理原始計畫、歷史診斷、批次報告與文件整理紀錄，不納入 Git。大型 raw samples、LiDAR、歷史 Logs、快取與完整研究批次留在原資料位置。正式文件引用的歷史路徑須標明「本機證據」，不能要求新讀者先取得它們才能理解成果。

每次新增驗收須記錄日期、輸入／幾何版本、命令、實際結果、限制及證據位置。回歸測試、離線重播、GUI 操作與閉迴路行車分別描述，未執行的檢查不可寫成通過。

正式指南在 `docs/project/`，可攜驗收報告與精簡圖表在 `docs/project/reports/`。`.gitignore` 明確納入這些文件，排除 `docs/local/`；納入規則不代表已提交或推送。
