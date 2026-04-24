对，这个会里**很有必要先把 API / 接口约定清楚**。
但要注意，你们现在要定义的不是所有函数，而是**模块之间的接口**。也就是：

> 谁给谁什么数据，格式是什么，多久给一次，出错怎么办。

你们这个项目最该先定好的，是下面这几类 API。

---

# 一、先定“公共基础接口”

这个最先定，不然后面每个人的数据格式都不一样。

## 1）统一时间戳

因为你们最后要做雷达和音频融合，所以每条数据都要带时间戳。文档里也强调过前期要做“统一时间戳日志”，这是必须先定的。

建议统一：

```c id="8a8ee"
typedef uint32_t timestamp_ms_t;
```

或者更完整一点：

```c id="9db5c2"
typedef struct {
    uint32_t timestamp_ms;
    uint8_t valid;
    int32_t err_code;
} base_meta_t;
```

---

## 2）统一状态码

所有模块都别各写各的返回值，统一成这种：

```c id="b0c4c5"
typedef enum {
    APP_OK = 0,
    APP_ERR_TIMEOUT,
    APP_ERR_INVALID_DATA,
    APP_ERR_NOT_READY,
    APP_ERR_OVERFLOW,
} app_status_t;
```

---

# 二、雷达驱动 API

这个是 B 必须先定下来的。

你们文档里写了 LD6002 通过 UART 接入，会输出 total phase、respiratory phase、heartbeat phase、呼吸率、心率，刷新 50 ms。

所以雷达模块至少要定义两层接口：

---

## 1）原始帧接口

这个接口面向驱动层。

```c id="048f80"
typedef struct {
    uint32_t timestamp_ms;
    float total_phase;
    float resp_phase;
    float heart_phase;
    float rr;
    float hr;
    uint8_t valid;
} radar_frame_t;
```

---

## 2）驱动层函数接口

```c id="81da9c"
app_status_t radar_init(void);
app_status_t radar_start(void);
app_status_t radar_stop(void);
app_status_t radar_get_latest_frame(radar_frame_t *frame);
app_status_t radar_is_ready(uint8_t *ready);
```

---

## 3）特征层接口

因为后面还要做雷达特征提取和模型，所以再定义一层窗口特征接口。你们文档里给的方向有 RR_mean/std、HR_mean、phase_amp 等。

```c id="18e6c0"
typedef struct {
    uint32_t timestamp_ms;
    float rr_mean;
    float rr_std;
    float hr_mean;
    float phase_amp;
    float motion_score;
    uint8_t valid;
} radar_feature_t;

app_status_t radar_extract_features(const radar_frame_t *frames,
                                    uint16_t frame_count,
                                    radar_feature_t *feat);
```

---

# 三、麦克风 / 音频采集 API

这是你这边最该先定好的。

文档里已经给了音频 baseline：**16 kHz、1 s 窗口、Log-Mel 40 维**，并且是双麦克风。

所以你们音频链路至少要先统一这几个问题：

* 输入是双通道还是单通道
* 1 次送多少样本
* 采样率固定多少
* 是送原始 PCM 还是送特征

---

## 1）音频窗口数据结构

```c id="65f8b7"
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_WINDOW_MS     1000
#define AUDIO_SAMPLES_PER_WINDOW 16000

typedef struct {
    uint32_t timestamp_ms;
    int16_t pcm[AUDIO_SAMPLES_PER_WINDOW];
    uint8_t selected_channel;   // 0/1
    uint8_t valid;
} audio_window_t;
```

如果你们后面决定保留双通道原始数据，也可以改成双声道版本。
但为了减轻耦合，建议先统一成：
**驱动层双通道采集，算法层先吃单通道选优后的窗口。**

这和你们文档里“先用双通道能量比较 + 选优通道”这个思路是一致的。

---

## 2）音频驱动接口

```c id="4ac9ba"
app_status_t audio_init(void);
app_status_t audio_start(void);
app_status_t audio_stop(void);
app_status_t audio_get_window(audio_window_t *win);
```

---

# 四、特征提取 API

这层特别重要，因为它正好卡在“驱动”和“模型”中间。

你们项目里：

* 音频侧要做 Log-Mel
* 雷达侧要做统计/相位特征
* DSP 会参与前处理和特征提取

所以建议单独把特征接口抽出来，不要塞进模型函数里。

---

## 1）音频特征接口

```c id="d2f6e2"
#define AUDIO_MEL_BINS 40

typedef struct {
    uint32_t timestamp_ms;
    float logmel[AUDIO_MEL_BINS][/*time_bins*/ 100];
    uint8_t valid;
} audio_feature_t;

app_status_t audio_extract_features(const audio_window_t *win,
                                    audio_feature_t *feat);
```

---

## 2）雷达特征接口

前面已经给了 `radar_feature_t`，保持统一即可。

---

# 五、模型推理 API

这个也必须提前定，因为以后不管你们是用 Ready Model、自训练 DS-CNN / CRNN，还是小型雷达模型，上层都不应该关心模型内部细节。文档里也已经规划了这些模型路线。

---

## 1）音频推理结果接口

```c id="c46c50"
typedef enum {
    AUDIO_EVT_NORMAL = 0,
    AUDIO_EVT_COUGH,
    AUDIO_EVT_WHEEZE,
    AUDIO_EVT_SNORE,
    AUDIO_EVT_NOISE
} audio_event_t;

typedef struct {
    uint32_t timestamp_ms;
    audio_event_t event;
    float probs[5];
    float confidence;
    uint8_t valid;
} audio_infer_result_t;

app_status_t audio_model_init(void);
app_status_t audio_model_infer(const audio_feature_t *feat,
                               audio_infer_result_t *result);
```

---

## 2）雷达推理结果接口

```c id="5814d5"
typedef struct {
    uint32_t timestamp_ms;
    float rr;
    float hr;
    float abnormal_score;
    float confidence;
    uint8_t valid;
} radar_infer_result_t;

app_status_t radar_model_init(void);
app_status_t radar_model_infer(const radar_feature_t *feat,
                               radar_infer_result_t *result);
```

---

# 六、融合 API

这个接口一定要开会时定下来。
因为你们项目最核心的亮点，本来就是“音频事件 + 雷达生理特征 → 两级决策融合”。

如果这个接口不先定，后面你和 B 各做各的，很容易最后接不上。

---

## 建议的融合结果接口

```c id="8b935d"
typedef enum {
    SYS_STATE_NORMAL = 0,
    SYS_STATE_COUGH_ALERT,
    SYS_STATE_WHEEZE_RISK,
    SYS_STATE_SNORE,
    SYS_STATE_ABNORMAL_BREATHING
} system_state_t;

typedef struct {
    uint32_t timestamp_ms;
    system_state_t state;
    float risk_score;
    char trigger_reason[64];
    uint8_t alarm_flag;
    uint8_t valid;
} fusion_result_t;

app_status_t fusion_init(void);
app_status_t fusion_update(const audio_infer_result_t *audio_res,
                           const radar_infer_result_t *radar_res,
                           fusion_result_t *fusion_res);
```

---

# 七、UI / 告警 API

这部分要给 C 定清楚。
不然 C 会不知道上层到底该接什么数据。

你们文档里后期会做 TFT 显示、语音提醒、手机 App 推送。

所以建议 UI 模块只接融合结果和少量摘要数据，不直接碰底层驱动。

---

## 1）UI 更新接口

```c id="414c9e"
app_status_t ui_init(void);
app_status_t ui_update_status(const fusion_result_t *fusion_res);
app_status_t ui_update_waveform(float rr, float hr, float motion_score);
```

---

## 2）告警 / 通知接口

```c id="4c6c39"
app_status_t notifier_init(void);
app_status_t notifier_voice_alert(const fusion_result_t *fusion_res);
app_status_t notifier_ble_push(const fusion_result_t *fusion_res);
```

---

# 八、日志 / 录制 API

这个很容易被忽略，但我建议你们一定要定。
因为后面训练、调试、答辩、演示都要靠日志。

```c id="78849d"
app_status_t log_init(void);
app_status_t log_radar_frame(const radar_frame_t *frame);
app_status_t log_audio_result(const audio_infer_result_t *result);
app_status_t log_fusion_result(const fusion_result_t *result);
```

---

# 九、配置 / 校准 API

这部分可以先简化，但最好占个位。
后面你们很可能会调：

* 雷达有效阈值
* 音频事件置信度阈值
* 告警门限
* UI 刷新开关

```c id="bd037f"
typedef struct {
    float cough_threshold;
    float wheeze_threshold;
    float radar_valid_threshold;
    float alarm_threshold;
} app_config_t;

app_status_t config_load(app_config_t *cfg);
app_status_t config_save(const app_config_t *cfg);
```

---

# 十、开会时最该定下来的，不只是 API 名字，而是这 6 件事

每个接口都要明确：

1. **输入是什么**
2. **输出是什么**
3. **单位和范围是什么**
4. **多久调用一次**
5. **失败时返回什么**
6. **谁负责维护这个接口**

例如：

* 雷达帧是 50 ms 一帧还是按需读取
* 音频是一秒一窗还是更短步长
* `rr` 单位是 bpm 还是 Hz
* `confidence` 范围是 0–1 还是 0–100

这些不写清楚，后面最容易扯皮。

---

# 十一、你们这次会最优先要敲定的 5 个接口

如果时间不够，不用一次把全部细节定死。
但这 5 个必须先定：

### 1. 雷达输出接口

`radar_frame_t / radar_feature_t`

### 2. 音频窗口接口

`audio_window_t`

### 3. 音频推理结果接口

`audio_infer_result_t`

### 4. 融合结果接口

`fusion_result_t`

### 5. UI 输入接口

`ui_update_status(...)`

这 5 个一旦定了，你们三个人就能并行开发。

---

# 十二、给你的一个实际建议

你这次会里不要追求把所有函数都写出来，
你只需要把**数据结构和模块边界**先定下来。

也就是先定：

* 结构体
* 枚举
* 返回值
* 调用关系

函数实现细节后面各自再补。

---

如果你愿意，我下一条可以直接帮你写一份**适合你们三人会议直接讨论的接口定义初稿（C 语言版）**。
