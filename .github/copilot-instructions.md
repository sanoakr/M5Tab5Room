# Copilot Chat Instructions (Project-wide)

このプロジェクトで Copilot Chat が従うルールと参照元です。**回答・コード例・コメントは日本語**で行ってください。

---

## 0) 前提 / 目的
- 対象ハードウェア: **M5Stack TAB5**
- SDK: **ESP-IDF v5.4.1 (esp32p4 ターゲット)**
- シェル: **fish**
- ビルド/書き込み: **`idf.py`**
- 依存: **Serena MCP**（本プロジェクトで利用。関連コード/API を前提に提案してください）
- 既存ソース: **`plathomes/tab5` 以下**を優先的に参照・再利用してください

---

## 1) 参照すべき公式ドキュメント
- M5Stack TAB5 仕様: <https://docs.m5stack.com/en/core/Tab5>
- ESP-IDF (esp32p4, v5.4.1) Get Started: <https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32p4/get-started/index.html>

> **重要:** 上記以外のサードパーティ資料や未確認ブログ等は根拠として使わず、まずは公式ドキュメントおよび既存ソース（`plathomes/tab5`）を参照してください。外部情報が必要な場合は、公式に当たる/比較する提案に留めてください。

---

## 2) リポジトリ構成の前提
- 既存コード、ドライバ、ユーティリティは **`plathomes/tab5`** 配下に集約されています。  
  - 新規コードを提案する場合は、**同階層/構成を尊重**し、**再利用できるヘッダ/モジュールがないか**先に探索する方針で。
  - 既存 API の命名・エラーハンドリング・ログ出力スタイルに合わせてください。

---

## 3) 開発ワークフロー（fish + idf.py）
- 環境ロード（fish）:
  ```fish
  # ESP-IDF を fish で有効化
  source $IDF_PATH/export.fish