# LMPC 核心文件

分支定位：本目錄隨 `research/lmpc` 納入版本控制；專案主線目前名為 `master`，以 overtaking 與其共用基礎為主。LMPC程式、方法與進度留在研究分支，raw實驗資料仍留本機；不將整個研究分支直接合回主線。

更新：2026-10-20（Asia/Taipei）。日常閱讀只維護三份核心內容，歷史進度及完整研究盤點另列。

| 文件 | 回答的問題 |
|---|---|
| [完成狀態](completed.md) | 做完哪些能力？目前哪些驗收還沒通過？ |
| [論文對照與運作方式](paper_method.md) | 控制流程怎麼運作？PDF、作者公開碼與本地變體有何差異？ |
| [最新進度：10/20](progress_20261020.md) | QP-D1、NO-FULL及四格比較實際得到什麼結果？ |

**目前仍無成功LMPC整圈。** 最新四格比較固定NO history與同一接手點：original／Reduced執行23步、前進6.988937m，其餘三組各6步；全部因QP迭代上限停止。NO消除本批過衝、Full縮短部分計算時間，但都未證明持續控制或整圈改善。不同歷史方法的最長片段不拿來排名。

## 完整報告與歷史

- [全實驗總整與作者流程比較](../../project/reports/experiment_synthesis_20261020.md)：完整變因、結果、問題證據分級及批次／session清冊；不是另一份每日狀態流水帳。
- [10/19作者QP／Safe Set細部對照](archive/20261020_document_cleanup/qp_implementation_comparison_20261019.md)：固定版本、函式與當時行號。
- [10/19進度](archive/20261020_document_cleanup/progress_20261019.md)、[10/15進度](archive/20261020_document_cleanup/progress_20261015.md)、[10/14進度](archive/20261020_document_cleanup/progress_20261014.md)：保留各日結論。
- [A0～A3細節](archive/20261020_document_cleanup/experiments_A0_A3_1005.md)：`1005`是原指定檔名，不是實驗日期。
- [本次封存清單](archive/20261020_document_cleanup/manifest.json)：移入5份歷史文件，保留4份核心文件整理前快照；含前後hash及驗證結果。舊26份紀錄仍在原 [封存清單](archive/manifest.json)。

## 維護規則

完成狀態直接重寫現況，不再在文首堆疊多段「最新」；方法文件維護穩定定義與差異；每天集中一份進度，以批次表區分離線、閉環與重播。舊進度的「目前／下一步」只指當時，原始Data報告不追改。

`checks/`保留重現及查核工具；[sources manifest](sources/r0_20261012/manifest.json)保留作者固定快照。raw資料不搬動。資料一致性、預測誤差、QP證書、實際控制、完整計算時間及學習各自判定。本目錄及整理報告由 `research/lmpc` 保存；其他本機筆記與大型Data維持忽略。公開發布以實際Git提交／推送為準。
