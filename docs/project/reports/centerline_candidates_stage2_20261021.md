# 第二階段：master 中線行駛、候選排程與同步記錄

日期：2026-10-21（Asia/Taipei）。**第二階段完成：在 master 執行同一次一圈 session，沿指定中線行駛並持續生成五條候選，獨立重播與來源／時間核對通過。** 尚未完成瀏覽器時間步播放或最後整體 UI 驗收。

## 設定與實測

本機完整資料：`Data/research/overtaking/stage2_esp_20261021_002/`（約 26 MB，raw JSONL 不納入 Git）。[獨立驗證 JSON](centerline_candidates_stage2_20261021.json)保存設定、原始串流 SHA256、執行時程式 hash、結果與接縫步號。

| 項目 | 實際結果 |
|---|---:|
| 分支 | master |
| 控制器 | 共用 CentrelinePurePursuit；候選不接管控制 |
| 速度／PP 前視 | 1.2 m/s／0.6 m |
| 物理／控制週期 | 0.01 s／0.1 s |
| 候選週期／前視／margin | 2 s 模擬時間／3 m／0.2 m，均固定 |
| 完整圈／reset／碰撞 | 1／1／無碰撞 |
| 圈時間估計 | 196.939487712 s |
| 實際記錄狀態／控制區間 | 1971／1970 |
| 規劃事件 | 100，含跨線後的終止事件 |
| 五 ID 結果總數 | 500 |
| 取樣通過／拒絕 | 391／109 |
| 每組通過候選數 | 2～5 |
| 最大側向誤差 | 0.081606615 m（門檻 0.25 m） |
| 最大航向誤差 | 0.194109255 rad（門檻 0.5 rad） |
| 通過候選最大起點位置誤差 | 4.34e-12 m（門檻 1e-4 m） |
| 通過候選最大起點航向誤差 | 1.78e-15 rad（門檻 1e-5 rad） |
| 規劃 median／P95／max 牆鐘時間 | 0.857373／0.975655／1.050387 s |
| 記錄迴圈牆鐘時間 | 87.094859 s，不含賽道載入與初始化 |
| 模擬秒／牆鐘秒 | 2.261902 |

拒絕的 109 條均保存 `optimizer_constraints_not_satisfied` 與求解嘗試診斷，不為湊足五條放寬限制。此處「通過」僅為 V5.2 中心點／輸出折線的取樣幾何與約束檢查；footprint_verified、continuous_verified、planning_allowed 均維持 false。新的中線實際行駛使用既有模擬器碰撞模型，沒有把歷史車身驗收套用到候選。

起終點完整跨圈位於控制步 1970；游標來源是該步實際車況，另產生一組終止候選，接縫不是將首筆畫面複製到結尾。起點微小負投影造成的首步 wrapped 回零不計為完整圈。

## 實作與驗證

- 只移入已驗證的 PP 類別與 core Frenet 圈追蹤，放在共用 core；沒有合併 research/lmpc 分支、修改原模擬動力學或引入 osqp。
- 新入口依序記錄 S_k、同步規劃、記錄 action_k 的 k→k+1 關係，再推進模擬。方案保存實際 x/y/yaw、來源／available 步號與時間、實際規劃參數、道路範圍及每條起點誤差。
- 無效來源、整組幾何拒絕與 runtime 中斷均有五個 ID；串流逐筆 flush，例外保存已完成記錄與 summary。若作業被直接 kill，仍可能沒有 summary／索引，JSONL 可供重建。
- 驗證器獨立重播 19,710 個物理子步與 steering buffer，重新計算全部 1970 個 PP 命令，重建 Frenet／圈數，核對五 ID、來源與時間、起點閘門、UTF-8 byte index 及倒序查找不顯示未來候選，全部通過。
- 四檔回歸（session、track_data、V5.2、Continuous）**56 passed，50.84 s**。最後補上無 Git 執行檔的版本讀取與完整跨圈判定後，session 檔再驗 **15 passed，16.84 s**；這些項目與前一批部分重複，不相加為獨立測試總數。
- session 記錄了 master 起始提交與執行時檔案 SHA256；當時尚未提交的實作由檔案 hash 識別。映像沒有 Git 執行檔，因此 tracked_diff_sha256 為 null，沒有假造工作樹 diff 識別；Git ref 由 checkout 的 ref 檔讀取。

## 重現與下一步

從 repository 根目錄，使用既有 f1tenth-sim:full Python 環境，指定新的輸出位置：

```bash
python -B -m f1tenth_benchmarks.research.run_centerline_candidates \
  --output-dir Data/research/overtaking/NEW_SESSION --laps 1 --plan-period 2 --lookahead 3
python -B -m f1tenth_benchmarks.research.overtaking.verify_session \
  Data/research/overtaking/NEW_SESSION --report NEW_VERIFICATION.json
```

同步規劃暫停模擬時間；本次效能量測不構成即時運算承諾。接著是第三階段瀏覽器時間步播放，再以同一整合 session 完成第四階段的操作與畫面驗收。尚未開始候選選擇、對手預測、超車閉迴路或實車部署。
