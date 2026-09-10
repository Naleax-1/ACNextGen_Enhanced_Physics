# ACNextGen — Definition Runtime Migration (Stage 1)

## 現在の状態 / Status

**完全再構築は未完了です。このPRは、旧版を既定で維持する第一段階の移行パイロットです。**

- ベースライン: `2f2ecb9`。既存 `modules/*.lua` 54ファイルは変更していません。
- ホストは53スロット（有効49）。存在しない `impact_state` は元から無効です。勝手に補完・有効化しません。
- `brake_fade` の18パラメータ、34状態フィールド、物理式、中間値、状態遷移、入力エイリアス順序、出力を `modules/brake_fade.json` に構造化しました。
- **移行済みDefinitionは1つです。** 他のモデル、`Car_Engine/` 分離、全体の少数Engineへの統合は未実装です。
- 原版ホストと対象モジュールを `Legacy/` に保存しています。未移行ソースは引き続き `modules/` で実行します。
- **車両の `data/` に `script.lua` を置きません。車両ファイルを生成・編集するインストーラーもありません。**

### 現時点の構成

```text
apps/lua/ACNextGen/
├─ ACNextGen.lua             # 元のスケジューラを維持する段階移行Host
├─ manifest.ini
├─ modules/
│  ├─ *.lua                 # 既存54ファイルを維持
│  └─ brake_fade.json       # 実行可能な構造化Definition
├─ Engine/
│  ├─ config.lua            # legacy / pilot の選択
│  ├─ migration.lua         # 移行期間のみの接続・起動時fallback
│  ├─ loader.lua            # 起動時発見・検証・依存/順序確認・キャッシュ
│  ├─ json.lua              # 起動時専用JSON decoder
│  ├─ compiler.lua          # 閉じた命令体系から実行関数を生成
│  ├─ brake.lua             # Brake domainの実行・状態所有
│  └─ result_bus.lua        # 結果、世代、エラー、観測用コピー
├─ Observer/observer.lua    # 新基盤の結果のみを表示
├─ Send/physics.lua         # ac.storeへの搬送のみ
├─ Legacy/                  # 比較用原版（通常はrequireしない）
├─ Migration_Catalog.json   # 全ソースの静的棚卸し。未抽出を明示
├─ tests/                   # 比較・結合・マイクロベンチマーク
└─ tools/inventory.py       # 棚卸し再生成/一致確認
```

### インストール・試験モード

このリポジトリのアプリファイルを、既存環境をバックアップしたうえで `apps/lua/ACNextGen/` に配置します。`Engine/`、`Observer/`、`Send/`、`modules/` の大文字小文字を保持してください。Pythonやテスト用ライブラリはゲーム内実行には不要です。

初期設定 `Engine/config.lua` は以下です。

```lua
return { mode = "legacy", observerHz = 20 }
```

`mode` を `"pilot"` に変更し、アプリを再起動すると、有効な `brake_fade.json` が元の45番スロット・10Hzで旧 `brake_fade.lua` の代わりに動きます。**同じスロットで新旧を二重実行しません。** 他のモジュールの周期、蓄積dt、0.050秒上限、catch-up制御は変更しません。

Definition編集後も再起動が必要です。フレーム中にJSONの再読込・再解析・再コンパイルは行いません。CSPの `io.scanDir` とLuaのコンパイル機能が利用できない場合は、パイロットを有効化せず、診断と旧実装を残します。実際のCSPバージョンとの互換性は実機確認が必要です。

元に戻すには `mode = "legacy"` に戻して再起動します。走行中の実装切替・状態リセットはしません。起動時にJSONが欠損・破損している場合だけ、既知の旧実装へfallbackします。実行時の例外はHostとResult Busに記録し、旧実装へ途中で切り替えません。途中まで公開された結果があり得るため、エラー中の結果は有効な新しい結果として扱わないでください。

### 責務と互換性上の注意

- DefinitionはLuaソース文字列ではなく、参照、二項演算、分岐、代入、実行段階、公開先などの構造です。起動時に検証済みノードからLua関数を生成し、実行中にDefinition木を解釈しません。
- `Send/physics.lua` は計算・補正を追加しません。ただし現在の搬送先は **`ac.store`によるアプリ用テレメトリ** です。タイヤ力・荷重・エンジントルクをAC本体へ直接適用するAPIではありません。物理への実反映完了とは主張しません。
- 元の `modules/physics.lua` は荷重正規化、加算、減衰を含むため、名前だけでSendへ移していません。
- 旧モジュールが `ac.load` で中間結果を読むため、試験モードでも公開順序と公開タイミングを維持します。全Sendをフレーム末尾に一括移動すると挙動が変わるため、この段階では行いません。
- `brake_lock`、`vehicle_condition` は `brake_fade` より後で更新されます。前回または最新の公開値を読むという依存を維持し、単純なトポロジカルソートで順序変更しません。
- 複数モジュールが同じキーを書くことがあります。`thermal`、`virtual_inertia`、`brake_system` などの共有キーも依存として記載しています。更新周期によって「最新の書き手」が変わります。
- 新しいObserverには、書込能力を持たないsnapshot関数だけを渡します。受け取るのはEngine状態・結果のコピーです。停止や例外がPhysics/Sendを止めないことを試験しています。新Observerの10/20/30/60Hz設定は表示側だけに適用します。
- 従来Observer/UIは互換性のため残しています。全Observerの新Result Busへの移行は未完了です。
- `Car_Engine`対象は `drivetrain.lua` 内のトルク曲線補間、fallbackトルク、エンジンブレーキ等です。LUT読込、filtered RPM、共有状態、トルク平滑化との境界を比較検証してから抽出します。試作品の別物理モデルは持ち込んでいません。

### Definitionの編集例

係数はJSONの `parameters` にあります。例えば `fadeStart`、`fadeEnd`、`minMu` を編集できますが、数値や式を変えれば物理挙動は変わります。基準値との同等性検証は再実行が必要です。

物理式の例（`temp - fadeStart` の構造表現）:

```json
{
  "op": "sub",
  "args": [
    { "temporary": "temperature" },
    { "parameter": "fadeStart" }
  ]
}
```

式ノードは数値・真偽値、`literal`、`dt`、`state`、`temporary`、`parameter`、`constant`、`op/args` を使います。車輪状態には `"wheel": true` を付けます。演算子は `add/sub/mul/div`、比較 `lt/le/gt/ge/eq/ne`、短絡演算 `and/or`、`min/max/num/clamp/not_nil/bool01` に限定します。実行命令は `set/value`、`read/into`、`branch/yes/no`、`call`、`foreach_wheel`、`publish`、`dt`、`stop` です。

- `formula` の各段階は `scope` と順序付き `steps` を持ちます。`entrypoints` が初期化・更新の開始点です。
- `inputs` の `selection: "lua_or"` は原版のエイリアス評価を保持します。0は真、falseは次候補へ進み、最後のfalseはnilと区別します。
- `state` は初期値と車輪単位かどうかを定義し、`intermediate` は一時値の名前を宣言します。一時値記述は現仕様では空オブジェクトです。
- `outputs` の式はEngineで評価され、その値だけがSendへ渡されます。
- 文法・参照・演算子・循環・スケジュールの検証はありますが、任意の変更の物理的妥当性や全経路の型安全性を証明するものではありません。

### テストの再実行

リポジトリ直下から実行してください。

```sh
# Lua単体の比較・検証（LuaJITまたはLua 5.4がインストール済みの場合）
luajit tests/run.lua
# または
lua5.4 tests/run.lua

# Python経由で両VM・実モジュール結合・ベンチマークを実行する場合
python -m pip install lupa==2.8
python tests/run.py
python tests/integration.py
python tools/inventory.py --check
python tests/benchmark.py
```

計測結果を保存する場合は `python tests/benchmark.py --output tests/benchmark_results.json` を使います。保存済み結果には測定時刻、実行環境、ソースSHA-256、各試行と中央値があります。コミット整理後は測定時コミットIDよりソースSHA-256で対象コードを確認してください。

比較許容誤差は `abs(a-b) <= 1e-12 + 1e-12 * max(abs(a), abs(b))` です。NaN/Infinityは原版と同じ分類で比較します。

検証範囲:

- 単体・基盤8スイート: 各VMで1,624,585項目を比較。初期化、BAD DT、閾値、熱回復、3000更新のseed固定replay、状態、参照/書込順序、API例外、Observer独立性、起動時限定I/Oを確認。
- 実モジュール結合: 原版と変更版を独立VMで実行。legacy/pilot/破損JSON/Observer例外の4ケースを両VMで各160フレーム比較。合計19,962,008項目を比較。
- 上記比較で観測された数値差は0でした。AC APIは模擬、車両LUT/INIは意図的に未提供、描画も模擬です。実CSP struct・実車データ・実走の保証ではありません。
- ベンチマークは `brake_fade` 単体、模擬 `ac.load/ac.store`、Observer無効、2000更新のwarm-up後に20000更新を7回交互測定します。起動コストは別計測です。
- メモリはLua heapの増分です。1000更新中の一時増分とGC後の保持増分を測ります。プロセスRSS、起動時常駐量、ピークメモリではありません。GC/JITにより負の増分もあり得ます。

**単体CPUの改善が見られても、アプリ全体のFPS改善やメモリ改善とは扱いません。実ゲームのCPU/FPS/メモリ測定は未実施です。**

### Production Gate / 未達項目

| 項目 | 現状 |
|---|---|
| 全Lua棚卸し | 54ソースのSHA-256、関数・状態・API・キー候補、53スロットを記録。静的候補であり全物理式の意味解析ではない |
| 全物理式のDefinition/JSON化 | **未完了**。`brake_fade`のみ |
| Engine / Car_Engine分類 | カタログ上の分類・境界候補。Car_Engine実装は未完了 |
| 依存・計算順序 | 元Host順序/周期を保持。対象Definitionを検証。全モデルの意味的依存解決は未完了 |
| State移行 | `brake_fade`のみ比較済み |
| 少数Engineへの統合 | Brake domain基盤のみ。全体統合は未完了 |
| Result Bus / Observer / Send | パイロット経路のみ接続。全体接続は未完了。Sendはtelemetryのみ |
| Error Recovery | 起動時検証/fallback・観測停止・API例外を模擬環境で確認。実機検証は未実施 |
| Old/New / Physics Integrity | 対象モデルと模擬入力での実モジュール結合を確認。全車両・実走同等性は未確認 |
| CPU / Memory | 単体マイクロベンチマークのみ |
| FPS / 実ゲームCPU・Memory | **未実施** |
| Legacy Runtime停止 | **未実施**。既定はlegacy。有効なpilotで対象スロットのみ置換 |
| Assetto Corsa実走確認 | **未実施** |

次は `brake_system` または `brake_lock` を1つずつ抽出し、同じ比較ゲートを通してBrake Engineへ追加します。その後、Car_Engine境界の抽出と他domainの統合を進めます。既存の `moduleDefs`、周期、書き手の優先関係を名前順でまとめ直してはいけません。ラボがなくても設計・抽出・ローカル比較は進められますが、最終Production Gateには実AC環境が必要です。

---

## Legacy project notes

以下は原版のプロジェクト紹介です。「complete」等の記述は今回の再構築完了を示しません。

## Overview

ACNeXtGen is an attempt to enhance and reinterpret part of Assetto Corsa’s physics and calculation behavior through Lua scripting.

This package is placed in:

```text
apps/lua/ACNextGen/
  ACNextGen.lua
  manifest.ini
  modules/*.lua
```

Its purpose is not to decorate the surface of the simulation, nor to simply add visual or force-feedback effects.
ACNeXtGen was created as a “root and trunk” approach: a project that studies the behavior of the car itself, then reshapes the way grip, load, compliance, drivetrain response, suspension input, tire memory, yaw behavior, and road information are interpreted.

By analyzing and editing the Lua scripts, anyone can touch the behavior of Assetto Corsa not only from the vehicle data folder, but from a more physical and systemic layer.

This project also works without placing `script.lua` inside each individual car.
That point is important. ACNeXtGen is intended to remain universal, modular, and independent from per-car edits as much as possible.

## Philosophy

ACNeXtGen was born from one belief:

A simulator should not only move a car.
It should give the driver a reason to understand the car.

The goal of this project is to help Assetto Corsa express more of the hidden work happening beneath the surface: the contact patch, the tire carcass, the load path, the driveline windup, the suspension delay, the recovery from slip, the yaw moment budget, and the dialogue between road, tire, chassis, and driver.

This is not a claim that ACNeXtGen is perfect.
Rather, it is a completed step — one possible contribution toward better physics, deeper vehicle behavior, and a more honest driving experience.

## Message

The One Point One Plan is now complete.

To everyone who opens these Lua files, reads them, edits them, breaks them, repairs them, and improves them:

Please do not treat physics as a closed box.

Please question it.
Please analyze it.
Please improve it.
Please search for what the car is really trying to say.

I sincerely hope this project becomes a small aid for those who pursue better physics, better simulation, and the truth hidden inside vehicle behavior.

May every user keep the power to seek truth.

## Credits

I took part in the creation and direction of this project, while the overall construction and refinement were supported with GPT-5.5.

Thank you for walking with ACNeXtGen.
And from here on as well — I look forward to what we will build next.

