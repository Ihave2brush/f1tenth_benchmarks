# 唯讀 Plan Binding 與雙車資料 Audit

這些工具只核對與整理保存資料，不執行候選生成、Selector、Prediction、Collision 或控制。
`contract_valid` 表示已實作的來源／格式檢查一致，不是物理模型正確或行車安全批准。
所有結果維持 `inspection_only=true`、`planning_allowed=false`。

## 模組與版本

- `research/overtaking/plan_binding_v1.py`：固定版本 polyline／五候選來源摘要。
- `research/overtaking/plan_binding_v2.py`：V1 內容加 analytic spline、origin、domain、geometry evidence。
- `tools/audit_two_car_contract_v1.py`：保存的 `frames.json`／`report.json` 唯讀來源 join 與 options 整理。

正式程式只需 Python 標準函式庫；V2 依賴 V1，Audit 依賴 V2/V1。
不需要其他研究分支的 Prediction／Collision／Adapter／Cost Input，也不接入 Selector。
測試中的雙車非干擾案例使用本 repository 既有模擬依賴。

## 使用

在專案根目錄執行，輸出必須是來源目錄之外的新目錄：

```bash
python -m tools.audit_two_car_contract_v1 --frames /data/case/frames.json --report /data/case/report.json --metadata /data/case/metadata.json --classification saved_two_car_simulation --output /data/audits/case_001
```

獨立 metadata JSON 至少明確提供 `geometry_id`、`frame_id`；可提供 `producer_commit`。
不能因 map 名稱或 XY 數字看似一致就猜 frame。原生雙車輸出沒有完整 metadata，
應由錄製方提供真實設定與來源，不從 CHANGELOG 補造。

可選 `--geometry-manifest` 和 `--expected-manifest-sha256`，後者必須來自可信的外部 pin。
沒有 pin 只能標示檔案與 manifest 自洽；即使 pin 匹配也不代表身分認證。
`map_sources_verified=false` 表示沒有驗證原始 map source bytes。檔案 hash 核對不解析或證明道路／曲線有效性。
`--parameters` 接受明確的 JSON 參數，不用 wheelbase 推估車長。

輸出 `audit.json` 與 `options.csv`；CSV 每格以 JSON 編碼，保留 null／bool／array。
有效資料契約 exit code 為 0，**不等於安全**。契約無效會保存診斷並回傳 2；
檔案、JSON 或其他執行錯誤仍會失敗。工具不覆寫既有輸出或來源。

## 語義與限制

- V1/V2 使用固定版本 typed JSON、排序 key、ordered arrays、有限 `float.hex()`，不四捨五入。
  int/float 與正負零保守視為不同內容，不可任意重新格式化數值後假定 digest 相同。
- V1 沒有涵蓋 spline；analytic consumer 必須使用 V2。舊 hash 規則沒有變更。
- Origin 優先使用存在的 `spline.source_s_origin`，否則使用 `reference_s_origin`。
- `computed_at_audit_not_producer_verified` 只代表本次收到的 Plan 內容；不是原生評估時的歷史 binding。
- C3／raceline 保留 reference 語義，不產生自車 modeled rollout。
- 同一 candidate 的不同速度保留不同 option index；`candidate_id=null` 的 raceline 不是五候選之一。
- signed 三圓淨距不是單圓 clipped clearance；缺少數值維持 null。
- requested horizon、`safe=true` 或 `first_rejection_s` 不能證明實際已查時域。
  未保存 checked range 時完整度 unknown；拒絕時間不是 TTC。
- `safe=true` 搭配 rejection reason/time 或與記錄門檻矛盾時回報資料契約錯誤。
- 檔案 hashes 與 canonical fingerprints 各有明確範圍；均不是簽章或安全證明。
- Audit 全批次載入／hash／複製，pinned geometry 目前會重複讀檔；不建議放進控制迴圈。

目前 producer model mapping 僅明確對照 `707d869073b8b1afe3d5a1d671e872581c01d4bb`。
未知版本保持未確認，不靜默套用相同物理語義。
沒有取得正式保存的 ESP 雙車 frames/report；測試 fixture 明確為 synthetic，
另有短程 my_map 模擬非干擾測試，不代表 ESP 長程或實車驗收。

## 測試

```bash
python -m pytest -q tests/research/test_plan_binding_v1.py tests/research/test_plan_binding_v2.py tests/research/test_two_car_contract_audit_v1.py
python -m pytest -q tests/research/test_two_car_candidates.py tests/research/test_three_disc_multi_car.py tests/research/test_candidate_footprint.py
```

固定 V1/V2 golden vectors 防止移植更改 canonical 契約。獨立 fixture 不借用未移植的研究模組。
巡航／跟車代表測試比較 Audit 前後的 selected ID、reason、actions、decision、costs、events 與車況。
Path Length Prototype 不屬於本次提交。

## 效能定位

本次參考量測：AMD EPYC 9V74 VM（5 個可用 CPU）、Python 3.9.25、NumPy 1.22.0、
SciPy 1.11.1；warm-up 3 次、重複 20 次，geometry 初始化不含於計時。
輸入為 ESP geometry 加人工 analytic plan（五候選、各一點／六係數），不是正式雙車錄製。

| 操作 | median ms | P95 ms | max ms |
|---|---:|---:|---:|
| V1 binding | 0.328 | 0.425 | 0.427 |
| V2 binding、不含 I/O | 1.172 | 1.421 | 1.495 |
| V2 binding、含 pinned artifact 檢查 | 2.984 | 3.375 | 3.599 |
| 一 frame Audit、不含 I/O | 2.938 | 3.741 | 3.756 |
| 一 frame Audit、含 pinned artifact 檢查 | 6.532 | 6.981 | 7.587 |

檔案讀取為 warm page cache；不同 VM、資料規模／快取與負載會改變結果。
301 點／候選的 dense V1/V2 binding 約 23–24 ms median，不能把單一起點的量測推廣到所有資料。
這些數值不是硬即時上界；本 PR 沒有新增控制迴圈呼叫。
