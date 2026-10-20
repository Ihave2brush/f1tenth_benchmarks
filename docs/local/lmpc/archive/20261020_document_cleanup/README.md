# LMPC 核心文件

更新：2026-10-20（Asia/Taipei）。日常閱讀從現行狀態、方法與最新每日進度開始；原始證據與歷史保留供追溯。

| 文件 | 內容 |
|---|---|
| [已完成內容與目前狀態](../../completed.md) | 已完成能力、各層驗收界線與接續工作 |
| [論文對照與運作方式](../../paper_method.md) | 六維paper基準及10/15各變體的明確差異 |
| [今日進度：2026-10-20](../../progress_20261020.md) | QP-D1：Full／Reduced八個case、原限制及KKT、獨立重現程式 |
| [10/19進度](progress_20261019.md) | 致動器修正、A5-NO、QP2診斷、S1失敗與當日證據索引 |
| [QP／Terminal Safe Set 原始實作對照](qp_implementation_comparison_20261019.md) | 論文、作者模擬、BARC公開碼與本地數值處理，附固定版本與行號 |

最新結論：A5-COMPARE四組新閉迴路均未完成圈；C00 original/Reduced為23步／6.988937m，C01 original/Full、C10 NO/Reduced、C11 NO/Full都是6步。四組history／起點／首輪原QP exact。共同前6步NO消除跨目標與多數輪角反向，Full縮短cycle，但整段預測RMSE與持續運行未改善。全部失敗QP已保存、487保護檔未變；不以舊run充比較組。詳見10/20進度最新段落。

## 歷史與細節

- [10/14進度](progress_20261014.md)：保留當日結果與QP加速證據。
- [10/15進度](progress_20261015.md)：A0～A8與M1／T1的歷史結果。
- [1005：A0～A3分組紀錄](experiments_A0_A3_1005.md)：10/15前半段細節；不是今天全部結果，也不代表10/05。
- [本次檢查紀錄](../../checks/document_review_20261015.json)：整理前222個當日文件／產物的來源、hash、可讀性與session核對。
- [10/19整理核對](../../checks/document_review_20261019.json)：今日文件索引與選定證據hash、連結及來源未變核對；不是重新執行實驗。
- 原有26份紀錄位於`archive/`，清單見 [manifest](../manifest.json)；其中「目前／下一步」均指原記錄時點。

## 維護方式

完成狀態集中更新`completed.md`，方法集中更新`paper_method.md`，每天一份進度；小型診斷保留在原Data目錄，不另開多份核心狀態文件。資料一致性、預測準確度、QP、真實控制、即時性與學習各自判定。

`checks/`保留重現／整理工具；[sources manifest](../../sources/r0_20261012/manifest.json)保留固定作者快照；raw資料仍在`Data/research/lmpc/`。本目錄沿用本機忽略範圍，本次沒有提交或推送。
