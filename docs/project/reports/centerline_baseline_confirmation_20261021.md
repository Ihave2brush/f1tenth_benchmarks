# 第一階段：既有中線行駛重新確認

日期：2026-10-21（Asia/Taipei）。結論：**第一階段重新確認通過，可沿用既有中線 Pure Pursuit 基礎；不需重建控制器。** 本次重新驗證歷史 session，沒有重新收集行車資料，也尚未接入五條候選。

## 來源與設定

來源為 `Data/research/lmpc/b1_esp_centerline_20261007_001/`，原始報告位於 research/lmpc 分支的 docs/local/lmpc/archive/b1_report_20261007.md；master 直接保存本篇摘要與重驗 JSON。雖然位於 lmpc 目錄，此 session 執行的是固定速度中線 Pure Pursuit，沒有執行 LMPC 學習或候選控制。

本次確認中線 CSV、manifest、地圖 YAML／影像及車輛參數與保存紀錄一致；geometry_id 為 `c53238c43d356997b7035bacb1907958cbc35d77d8f8916ec4691ca9c934de71`，同本次 TrackData 所載入的 ESP 中線。

| 項目 | 設定／結果 |
|---|---|
| 控制器 | 既有 `research/lmpc/initialization.py::CentrelinePurePursuit` |
| 進度與圈數 | 既有 `CoreLapTracker`，使用 core Frenet；非舊 raceline 進度 |
| 速度目標／PP 前視 | 1.2 m/s／0.6 m，起步不使用原版 4 m/s 分支 |
| 物理／控制週期 | 0.01 s／0.1 s |
| 完整圈／reset | 2／1，跨線不 reset |
| 圈時間 | 196.939487712 s、196.722210590 s |
| 最大側向誤差 | 0.082636415 m，原門檻 0.25 m |
| 最大航向誤差 | 0.197707620 rad，原門檻 0.5 rad |
| 車身檢查 | 長 0.58 m、寬 0.31 m，各側加 0.05 m 餘裕；每物理子步 pre/post 矩形與阻擋格重疊檢查 |

## 2026-10-21 重新執行的驗證

[獨立重驗報告](centerline_baseline_reverification_20261021.json) 是本次執行現行 `verify_initialization` 的輸出，不是複製歷史 passed 旗標：

- 3,938 筆實際車況、39,380 個原模型物理子步重播一致，容差 1e-12。
- Frenet、圈事件、時間、實際輸入、單次 reset、追蹤界限與終止樣本關係均通過重建核對。
- 78,760 次取樣車身檢查全部通過。這是物理子步端點取樣，並非連續時間 swept-volume 證明，也不驗證超車候選。

[來源與控制命令報告](centerline_baseline_sources_20261021.json) 另完成：

- 用新 TrackData 載入的同一條中線交給現行 CentrelinePurePursuit，從每筆前一車況重新計算 3,937 次控制命令；與保存命令的最大差為 **0**（核對容差 1e-12）。
- 兩次終點跨越均為 s_wrapped 回零、s_unwrapped 正向連續；第一圈跨線後的下一筆實際物理狀態連續。最後一圈在跨線當步結束。
- 部分歷史程式檔與現行檔案 hash 不同，報告逐檔保存差異；不能宣稱完整歷史程式位元一致。此次以現行控制命令重算與獨立原模型重播驗證相容性。
- 報告保存原始 session／samples／events／歷史 verification 的 SHA256，以及本次額外核對腳本全文，供重現與追溯。原始檔案未修改。

既有回歸執行：

```bash
bash tests/run_research_tests.sh \
  tests/research/test_lmpc_interfaces.py \
  tests/research/test_lmpc_initialization.py \
  tests/research/test_continuous_laps.py -q
```

結果為 **29 passed、2 failed**（5.67 s）。兩個失敗項目是 LMPC A0/A1 與 A2 設定測試，載入 `run_lmpc` 時缺少 `osqp`；不是中線追蹤或重播失敗。此映像的基礎 CI 依賴未包含該 solver；本次未擴充 LMPC 環境。中線控制、共用進度、車身及 Continuous 生命週期相關測試均通過。不得把此結果寫成整組 31 項全數通過。

以下為原 research/lmpc 分支上的歷史重驗命令，master 不要求存在 LMPC 模組；master 新整合 session 請使用 overtaking.verify_session。歷史重驗輸出須使用新路徑：

```bash
python -B -m f1tenth_benchmarks.research.lmpc.verify_initialization \
  Data/research/lmpc/b1_esp_centerline_20261007_001 \
  --output NEW_VERIFICATION.json
```

在 research/lmpc 分支上，來源報告中的額外核對可將 `audit_source` 存成 `.py`，在相同 repository 根目錄與既有模擬 Python 環境執行；先將輸出的 `out` 改為新路徑，避免覆寫。

## 下一階段邊界

第一階段完成的是既有基礎的重新確認，沒有新增執行期程式。第二階段沿用上述 PP、core Frenet 與 Continuous 生命週期，將共用 TrackData、候選排程及來源車況記錄接進同一次新 session；若調整速度、前視或控制頻率，須再跑對應回歸。

最終仍需一份新的整合 session，至少一完整圈，期間候選持續更新並覆蓋接縫，才可完成最後整體驗收。幾何 planning_allowed／domain_verified／continuous_verified 不因本次通過而改為 true。
