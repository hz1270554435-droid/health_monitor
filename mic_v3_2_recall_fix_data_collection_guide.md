# MIC v3.2 Recall-Fix 数据采集使用指南

本文档用于指导 `MIC v3.2 recall-fix` 的板端实录音频采集。目标是收集
当前人的咳嗽召回修复数据，同时保留 v3.1-fpfix 已经得到的低误报优势。

本指南只覆盖数据采集和采集后准备命令，不训练模型，不改
`ml/data/raw/`，不覆盖旧标签文件。

相关计划文档：

```text
ml/review/v3_2_recall_fix/v3_2_recall_fix_plan.md
ml/review/v3_2_recall_fix/v3_2_commands_draft.md
```

## 采集规则

- 新录音统一放在外部数据目录：`D:\cough_model_train\DATA\...`
- 不要把录音复制到 `ml/data/raw/`。
- 前处理目标保持 `board_htk_no_norm_v1`。
- v3.1-fpfix ONNX/模型分数只能用于筛选和复听排序，不能直接当标签。
- 只有人工复听后的 `manual_decision` 可以进入候选标签。
- 采集阶段不要训练模型。
- 采集阶段不要做 OPERA / HeAR / YAMNet 蒸馏。
- `eval_current_person` 和 `smoke_negative` 是后续评估/冒烟集合，不进入训练集。

## 目录准备

在顶层工作区执行：

```powershell
Set-Location D:\e84_health_monitor

$Date = Get-Date -Format yyyyMMdd
$DataRoot = "D:\cough_model_train\DATA\board_live_v3_2_recall_fix_$Date"

New-Item -ItemType Directory -Force -Path `
  "$DataRoot\train_candidate", `
  "$DataRoot\eval_current_person", `
  "$DataRoot\smoke_negative", `
  "$DataRoot\notes"
```

目标目录结构：

```text
D:\cough_model_train\DATA\board_live_v3_2_recall_fix_YYYYMMDD
  train_candidate\
    sessions\audio\*.wav
    sessions\audio\*.json
  eval_current_person\
    sessions\audio\*.wav
    sessions\audio\*.json
  smoke_negative\
    sessions\audio\*.wav
    sessions\audio\*.json
  notes\
```

三个子集的用途：

| 子集 | 用途 | 后续处理 |
| --- | --- | --- |
| `train_candidate` | v3.2 训练候选数据，后续需要人工复听 | 复听确认后才可进入候选标签 |
| `eval_current_person` | 当前人咳嗽召回评估 | 必须留在训练外 |
| `smoke_negative` | 新鲜非训练负样本冒烟集 | 必须留在训练外 |

## 固件准备

v3.2 recall-fix 当前只需要 MIC 数据，优先使用 CSV_EXPORT audio-only 固件。

在顶层工作区构建：

```powershell
Set-Location D:\e84_health_monitor
. .\tools\enter_mtb_env.ps1
make firmware-csv-audio
```

如果板子还没有烧录这个构建，再手动烧录：

```powershell
make -C firmware program
```

说明：

- `make firmware-csv-audio` 对应 `APP_RUNTIME_MODE=1` 和
  `APP_CSV_EXPORT_CAPTURE_MODE=0`。
- Debug UART 波特率使用 `2000000`。
- 本批数据默认不要用 audio+radar，因为 radar 文本流会增加高频 MIC
  采集干扰风险。只有明确要采同步 fusion/radar 数据时才用
  `make firmware-csv-audio-radar`。

## 查找串口

在顶层工作区执行：

```powershell
Set-Location D:\e84_health_monitor
python firmware\tools\capture_csv.py --list-ports
```

也可以使用 firmware 工具目录下的 Windows wrapper：

```powershell
Set-Location D:\e84_health_monitor\firmware\tools
.\run_capture_csv.bat --list-ports
```

设置串口变量：

```powershell
$Port = "COM7"
```

如果不确定端口，可以先尝试：

```powershell
$Port = "auto"
```

如果采集后没有 `PCMB` 数据，改用明确的 KitProg USB-UART 端口，例如
`COM7`。

## capture_csv.py 常用参数

采集脚本：

```text
firmware\tools\capture_csv.py
```

常用参数：

| 参数 | 推荐值 | 说明 |
| --- | --- | --- |
| `--port` | `$Port` | 串口，例如 `COM7` 或 `auto` |
| `--baud` | `2000000` | Debug UART 波特率 |
| `--duration` | 45 / 60 / 90 | 本次采集持续秒数，`0` 表示直到 Ctrl+C |
| `--output-dir` | `$DataRoot\<subset>` | 当前子集输出根目录 |
| `--session-output-dir` | `sessions` | 训练 session 输出目录 |
| `--audio-session-sec` | 45 或 60 | 每个 WAV session 长度 |
| `--person-id` | `p01` | 当前采集人的 person_id |
| `--scene` | 场景名 | 写入 JSON，后续 manifest 会读取 |
| `--distance-cm` | 40 等 | 距离，写入 JSON |
| `--audio-fs` | `16000` | WAV 采样率 |
| `--audio-channels` | `1` | 导出单通道 WAV |
| `--audio-mix-mode` | `select-best` | 使用类似固件能量选择的通道策略 |

输出路径规则：

```text
--output-dir "$DataRoot\train_candidate" --session-output-dir sessions
```

会生成：

```text
$DataRoot\train_candidate\sessions\audio\session_0001_..._audio.wav
$DataRoot\train_candidate\sessions\audio\session_0001_..._audio.json
```

不要加 `--expand-binary-audio-csv`，除非你明确需要逐采样 CSV。它会生成很大的 CSV。

## 通用采集模板

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration <采集秒数> `
  --output-dir "<输出子集目录>" `
  --session-output-dir sessions `
  --audio-session-sec <每个session秒数> `
  --person-id p01 `
  --scene "<场景名>" `
  --distance-cm <距离cm> `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

如果 `--duration 0`，用 `Ctrl+C` 结束。脚本会尽量关闭当前 WAV 并写出
对应 JSON。

## 采集数量建议

| 类别 | 最低建议 | 更推荐 | 备注 |
| --- | ---: | ---: | --- |
| `cough` | 8 段 / 约 40 次咳嗽 | 12-16 段 / 60-90 次咳嗽 | 重点覆盖当前人 40cm 弱召回场景 |
| `quiet` | 4 x 60s | 6 x 60s | 覆盖低能量安静环境 |
| `speech` | 4 x 60s | 6 x 60s | 普通讲话和大声讲话都要有 |
| `throat_clear` | 3 x 45s | 4-6 x 45s | 清嗓是边界负样本 |
| `knock` | 3 x 45s | 4-6 x 45s | 桌面/设备敲击 |
| `handling_noise` | 3 x 45s | 4-6 x 45s | 拿起、放下、摩擦、线缆/外壳触碰 |

建议把当前人的咳嗽分成两部分：

- `train_candidate`：可用于后续人工复听和候选训练。
- `eval_current_person`：必须留作训练外召回评估。

## 分场景采集命令

以下命令都从顶层工作区执行：

```powershell
Set-Location D:\e84_health_monitor
$Port = "COM7"
$Date = Get-Date -Format yyyyMMdd
$DataRoot = "D:\cough_model_train\DATA\board_live_v3_2_recall_fix_$Date"
```

### 1. 训练候选：当前人 40cm 咳嗽

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 90 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 60 `
  --person-id p01 `
  --scene "cough_40cm_front_normal" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

采集动作建议：

- 距离板子约 40cm。
- 正对麦克风。
- 包含轻咳、正常咳、连续咳。
- 每段记录大约 5-8 次咳嗽。

### 2. 训练候选：安静房间

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 60 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 60 `
  --person-id p01 `
  --scene "quiet_room" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 3. 训练候选：普通讲话

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 60 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 60 `
  --person-id p01 `
  --scene "speech_normal" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 4. 训练候选：大声讲话

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 60 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 60 `
  --person-id p01 `
  --scene "speech_loud" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 5. 训练候选：清嗓

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 45 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 45 `
  --person-id p01 `
  --scene "throat_clear" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 6. 训练候选：敲击

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 45 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 45 `
  --person-id p01 `
  --scene "knock" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 7. 训练候选：设备/线缆触碰噪声

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 45 `
  --output-dir "$DataRoot\train_candidate" `
  --session-output-dir sessions `
  --audio-session-sec 45 `
  --person-id p01 `
  --scene "handling_noise" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 8. 留出评估：当前人咳嗽

这部分不能进入训练集。

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 90 `
  --output-dir "$DataRoot\eval_current_person" `
  --session-output-dir sessions `
  --audio-session-sec 60 `
  --person-id p01 `
  --scene "eval_current_person_cough_40cm" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

### 9. 留出冒烟负样本

对每个负样本场景至少采 2 段，场景名建议使用：

```text
smoke_negative_quiet
smoke_negative_speech
smoke_negative_throat_clear
smoke_negative_knock
smoke_negative_handling_noise
```

示例：

```powershell
python firmware\tools\capture_csv.py `
  --port $Port `
  --baud 2000000 `
  --duration 60 `
  --output-dir "$DataRoot\smoke_negative" `
  --session-output-dir sessions `
  --audio-session-sec 60 `
  --person-id p01 `
  --scene "smoke_negative_speech" `
  --distance-cm 40 `
  --audio-fs 16000 `
  --audio-channels 1 `
  --audio-mix-mode select-best
```

## 采集记录

建议每天创建一个记录文件：

```powershell
notepad "$DataRoot\notes\collection_notes_$Date.md"
```

推荐模板：

```text
# v3.2 recall-fix collection YYYYMMDD

Board:
Firmware build:
Port:
Room:
Mic position:
Person:

Sessions:
- train_candidate cough_40cm_front_normal:
  start:
  details:
  issues:
- train_candidate quiet_room:
  start:
  details:
  issues:
```

这些 notes 只作为采集背景，不作为正式标签。

## 检查采集输出

列出 WAV：

```powershell
Get-ChildItem -Path "$DataRoot\train_candidate\sessions\audio" -Filter "*_audio.wav" |
  Select-Object Name,Length,LastWriteTime
```

列出 JSON：

```powershell
Get-ChildItem -Path "$DataRoot\train_candidate\sessions\audio" -Filter "*_audio.json" |
  Select-Object Name,Length,LastWriteTime
```

预期：

- 每段 session 有一个 `.wav` 和一个 `.json`。
- WAV 文件非 0 字节。
- WAV 采样率应为 16000 Hz。
- JSON 中应有 `person_id`、`scene`、`distance_cm`、`fs`、`channels`、
  `audio_mix_mode` 等信息。

如果要快速检查三个子集数量：

```powershell
Get-ChildItem -Path "$DataRoot\train_candidate\sessions\audio" -Filter "*_audio.wav" | Measure-Object
Get-ChildItem -Path "$DataRoot\eval_current_person\sessions\audio" -Filter "*_audio.wav" | Measure-Object
Get-ChildItem -Path "$DataRoot\smoke_negative\sessions\audio" -Filter "*_audio.wav" | Measure-Object
```

## 采集后生成 Window Manifest

采集完成后，进入 ML 仓库：

```powershell
Set-Location D:\e84_health_monitor\ml
$Date = Get-Date -Format yyyyMMdd
$DataRoot = "D:\cough_model_train\DATA\board_live_v3_2_recall_fix_$Date"
```

训练候选 manifest：

```powershell
python scripts\build_v3_1_window_manifest_from_sessions.py `
  --session-dir "$DataRoot\train_candidate\sessions\audio" `
  --out review\v3_2_recall_fix\window_manifest_train_candidate.csv `
  --source-domain board_live_v3_2_recall_fix `
  --scenario-hint unknown `
  --window-sec 1.0 `
  --hop-sec 0.5
```

当前人留出评估 manifest：

```powershell
python scripts\build_v3_1_window_manifest_from_sessions.py `
  --session-dir "$DataRoot\eval_current_person\sessions\audio" `
  --out review\v3_2_recall_fix\window_manifest_eval_current_person.csv `
  --source-domain board_live_v3_2_recall_fix_eval_current_person `
  --scenario-hint unknown `
  --window-sec 1.0 `
  --hop-sec 0.5
```

新鲜负样本冒烟 manifest：

```powershell
python scripts\build_v3_1_window_manifest_from_sessions.py `
  --session-dir "$DataRoot\smoke_negative\sessions\audio" `
  --out review\v3_2_recall_fix\window_manifest_smoke_negative.csv `
  --source-domain board_live_v3_2_recall_fix_smoke_negative `
  --scenario-hint unknown `
  --window-sec 1.0 `
  --hop-sec 0.5
```

每个 manifest 旁边会生成 report，例如：

```text
review\v3_2_recall_fix\window_manifest_train_candidate_report.json
```

## 可选：YAMNet 弱打分

YAMNet 只用于复听排序，不是标签来源。

```powershell
python scripts\scan_yamnet_v3_1_audio.py `
  --config configs\audio_mining_v3_1.yaml `
  --manifest review\v3_2_recall_fix\window_manifest_train_candidate.csv `
  --out review\v3_2_recall_fix\yamnet_scored_manifest_train_candidate.csv `
  --report review\v3_2_recall_fix\yamnet_scan_train_candidate_report.json `
  --summary-md review\v3_2_recall_fix\yamnet_scan_train_candidate_summary.md
```

后续继续看：

```text
ml/review/v3_2_recall_fix/v3_2_commands_draft.md
```

注意：当前还缺一个可复用的 v3.1-fpfix ONNX window scorer。没有这个脚本前，
不要把 checkpoint fallback 的分数当作最终部署 ONNX replay 结果。

## 不要在采集阶段运行

采集阶段不要运行：

```text
python scripts\train_audio_baseline.py ...
python scripts\run_fpfix_train_smoke.py ...
python scripts\run_fpfix_formal_train.py ...
python scripts\export_audio_model.py ...
python scripts\export_audio_model_pt2.py ...
```

也不要覆盖：

```text
ml\data\labels\audio_labels_v3.csv
ml\data\labels\audio_labels_v3_1_fpfix_train.csv
```

## 常见问题

没有生成文件：

1. 确认板子已烧录 CSV_EXPORT audio-only 固件。
2. 运行 `python firmware\tools\capture_csv.py --list-ports`。
3. 用明确的 `--port COMx`，不要先依赖 `auto`。
4. 确认 `--baud 2000000`。
5. 检查板子供电和 KitProg USB-UART 连接。

没有 `PCMB` 数据：

1. 多半是串口选错或固件不是 CSV_EXPORT audio-only。
2. 重新运行 `make firmware-csv-audio`，必要时 `make -C firmware program`。
3. 重新采一段 10 秒短样本验证。

WAV 太短或静音：

1. 确认 `--duration` 不小于 `--audio-session-sec`。
2. 使用 `--audio-channels 1 --audio-mix-mode select-best`。
3. 检查麦克风方向、距离和板子是否真的在输出 PCMB。

Python 提示缺少 serial：

```powershell
python -m pip install pyserial
```

## 下一步

采集完成并生成三个 manifest 后：

1. 保存采集 notes。
2. 先不要训练。
3. 补齐或运行 v3.1-fpfix ONNX window scorer。
4. 生成 review plan 和 review clips。
5. 人工复听填写 `manual_decision` / `manual_subtype`。
6. 只有人工确认后的 candidate 才能进入后续 v3.2 train manifest。

