# Offline Cost Evaluation V1

本模組是離線研究工具，維持 `inspection_only=true`、`planning_allowed=false`。

## 相依與範圍

本分支依賴 PR #10 的 Plan Binding V1/V2 與 Two-Car Audit；待該 PR 合併後才適合合併本 Draft。
相對 PR #10，新增三個模組、三個專屬測試及本文件，不帶入 Prediction、Collision、Cost Input、Rollout Diagnostics 或 E2E 成果。
不修改 Selector、PP、物理模型、三圓碰撞公式或候選生成器。

## API

- `evaluate_path_length_features(plan, geometry, *, binding=None, session_metadata=None, ...)`：計算 XY 折線長度與 Frenet 積分，記錄 origin、sampling domain 與 geometry 證據。
- `build_cost_features(audit_result, path_length_results, *, risk_sample_series=None)`：依 candidate/option identity 配對保存診斷，不重新執行預測、碰撞或生成器。
- `evaluate_costs(features, *, policy=None)`：保留長度、接近風險與相對進度的獨立分項；沒有預設權重、排名或選路。

## 指標與限制

1. Path length：候選曲線幾何長度。不同終點未必具有公平可比較性；C3 reference length 不等於 modeled ego travel length。
2. Proximity：保存 signed 三圓最小淨距離；只有提供有效時間序列時才能計算梯形 exposure integral。單一 minimum 不能推導時間積分。
3. Relative progress：沿用 native relative_s_progress_m 的原始估計語義，不宣稱是完整 ego unwrapped s(t) 積分。

未知、缺值、原生 unsafe/rejected、reference_only、來源或時域不完整不得自動變成安全或低成本。
明確政策須記錄量綱、尺度、範圍；只有具備比較條件與明確實驗政策才允許 experimental 綜合分數。本版不選最低分。
Binding 僅證明契約範圍內內容一致；不證明 producer 歷史、可行駛或安全。
尚未支援正式 ESP 雙車成本比較；目前數學驗證主要使用 synthetic fixtures。

## 驗證

```bash
python -m pytest -q tests/research/test_path_length_feature_v1.py tests/research/test_cost_feature_v1.py tests/research/test_cost_evaluation_v1.py
```

請審查 candidate-speed identity、比較目標／終點、producer binding 與 checked horizon 的未來介面；本次不接入線上決策。
