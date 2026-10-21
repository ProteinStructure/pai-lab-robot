

\# AI-Powered Laboratory Sample Transport



\## 概要



PythonとPygameを用いて、研究室内で検体を搬送する移動ロボットを模擬する2Dシミュレーションです。



\## 主な機能



\- A\*探索アルゴリズムによる経路計画

\- 障害物を考慮した経路探索

\- 検体棚から分析装置までの自動搬送

\- 搬送状態の画面表示



\## 使用技術



\- Python 3.13

\- Pygame

\- A\*探索アルゴリズム



\## 実行方法



Windows PowerShellでプロジェクトのフォルダに移動し、次のコマンドを実行します。



```powershell

python -m venv .venv

.\\.venv\\Scripts\\python.exe -m pip install -r requirements.txt

.\\.venv\\Scripts\\python.exe main.py

```



\## 操作方法



\- `R`：タスクを最初から実行

\- `ESC`：シミュレーションを終了



\## 実装上の注意



本プロジェクトは2Dのシミュレーションです。

検体の回収・搬送は状態の切り替えによって表現しており、物理的な把持動作は実装していません。