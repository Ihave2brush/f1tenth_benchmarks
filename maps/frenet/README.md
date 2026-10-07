# 內建地圖 Frenet 資料

本目錄納入 Git，保存五張內建圖（ESP、AUT、GBR、MCO、CornerHall）與實際 PGM 測試圖 my_map 的轉換資料。
原始 PNG、YAML 與原版中線保留在上一層 `maps/`，供原 benchmark 使用。

```text
maps/frenet/
├── inspector_catalog.json
├── publication.json
└── <地圖>/
    ├── geometry/
    │   ├── centerline.csv
    │   ├── boundaries.csv
    │   ├── reference_curve.json
    │   ├── waypoints_ros.csv
    │   ├── manifest.json
    │   ├── validation.json
    │   └── validity.json
    └── ranges/
        ├── section_ranges.json.gz
        ├── report.json
        └── manifest.json
```

六張地圖使用相同層級；geometry 保存可重載的中線與射線邊界，ranges 保存
兩種 d 範圍、全部樣本與限制原因。CSV 只放在 geometry，ranges/manifest.json 以
csv_location=source_geometry 指向同一份 CSV，保留來源／檔案雜湊。gzip 是無損壓縮，檢視器直接讀取，不需手動解壓。

從 repository 根目錄啟動：

```bash
python -B -m f1tenth_benchmarks.research.core.geometry.viewer \
  --catalog maps/frenet/inspector_catalog.json --port 8765
```

`publication.json` 核對正式檔案；每圖 ranges/manifest.json 保留原數值匯出
程式雜湊、來源 bundle_id 與未壓縮內容雜湊，重新計算壓縮檔案及包裝識別。
移動及壓縮不改變 geometry_id、CSV、座標或數值驗證結果。

CornerHall 是從下端向上再向左的開放走廊，其 s∈[0,L]；其他五張（ESP、AUT、GBR、MCO、my_map）為閉合賽道。
通過的是有限採樣與查詢目標，continuous_verified、planning_allowed 仍為 false。
左右邊界 CSV 是各截面法線首次命中牆面的點，不保證可直接連成物理牆線。
候選仍須精確查詢，後續在 x/y 檢查車身與動態。

研究批次、歷史版本、圖像及完整驗收紀錄留在本機
`Data/research/geometry/archive/2026-10-07_before_publication/`，不納入本目錄。
新研究輸出也先放在 `Data/research/geometry/`；驗證後用 publish_tracks 工具
建立新的候選發佈目錄，再檢查差異，避免直接覆蓋此版本。


## 實際 PGM：my_map

原始影像為 `maps/my_map.pgm`，工作設定為 `maps/my_map.yaml`；原始上傳 YAML
保留在 `maps/my_map.original.yaml`。依使用者確認，灰階205為未知區；原本
free_thresh=0.25會誤判為free，工作設定修正為0.196，影像／resolution／origin不變。
來源與設定調整記錄在 `maps/my_map.import.json`。

my_map使用相同 equidistant_periodic_v1，中線11.930043 m，起點位於下方較平直區段
約(0.880876,−2.437139)，正向逆時針。239列三線座標、62個截面、2,747範圍樣本
通過；所選賽道5,204格中心點往返通過，另外7個孤立free格標為outside_track。
已驗證樣本未發現拒絕；仍是有限採樣，不代表每個連續點或車身均已驗證。
