/******************************************************************************
* File Name : app_pdm_pcm.c
*
* Description : PDM 转 PCM 采集模块。
*               ISR 从 PDM/PCM 硬件 FIFO 读取左右声道数据，拼成固定 10 ms
*               PCM block；推理任务通过队列拿到 block 描述符后再处理数据。
*               队列不搬运 PCM 本体，只传递指针、长度和 block 索引。
*
*******************************************************************************/

/*******************************************************************************
* 头文件
*******************************************************************************/
#include "app_pdm_pcm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/*******************************************************************************
* 私有状态与共享缓冲
*******************************************************************************/
/* PDM/PCM 中断配置参数：这里使用右声道 IRQ 作为左右声道同步读取的触发源。 */
const cy_stc_sysint_t PDM_IRQ_cfg = {
    .intrSrc = (IRQn_Type)PDM_IRQ,
    .intrPriority = PDM_PCM_ISR_PRIORITY
};

typedef enum
{
    /* 该块空闲，已经归还给空闲池，ISR 可以重新写入。 */
    APP_PDM_PCM_BLOCK_FREE = 0,
    /* ISR 当前正在向该块追加 PCM 采样，此时消费者不能访问。 */
    APP_PDM_PCM_BLOCK_FILLING,
    /* 该块已经入队给消费者，消费者 release 前不能被覆盖。 */
    APP_PDM_PCM_BLOCK_READY
} app_pdm_pcm_block_state_t;

/* 面向推理任务的 10 ms 双声道音频块环形缓冲区。
 * 该缓冲区放在共享 SoC 内存中，后续如果把推理放到 CM55，
 * 可以直接消费同一批 PCM 块，避免从 CM33 本地 SRAM 再复制一份。
 */
int16_t recorded_data[APP_PDM_PCM_BLOCK_COUNT][APP_PDM_PCM_BLOCK_SAMPLES]
    __attribute__((section(".cy_shared_socmem"))) = {0};

/* 兼容旧调试逻辑的“当前写指针”和“当前 block 已写入长度”。
 * 新的推理链路不要直接依赖这两个变量，应该通过 app_pdm_pcm_receive_block()
 * 获取完整 block，避免读到 ISR 正在写入的半成品数据。
 */
int32_t recorded_data_size;
volatile int16_t *audio_data_ptr = NULL;

/* 队列只传递很小的块描述符，不复制原始 PCM 数据。 */
static QueueHandle_t pdm_pcm_block_queue = NULL;
static TaskHandle_t pdm_pcm_task_handle = NULL;

/* 采集流状态：
 * block_state 标记每个 block 的所有权；
 * free_block_stack 是 O(1) 空闲块池，避免 ISR 每次都扫描整个环形数组；
 * write_* 指向当前 ISR 正在填充的 block；
 * dropped_block_count 记录背压导致的丢块，便于在测试日志里观察推理是否跟不上。
 *
 * 消费者只能通过 app_pdm_pcm_release_block() 归还块，
 * 这样可以明确每个 block 当前归谁所有。
 */
static volatile app_pdm_pcm_block_state_t block_state[APP_PDM_PCM_BLOCK_COUNT];
static uint8_t free_block_stack[APP_PDM_PCM_BLOCK_COUNT];
static volatile uint8_t free_block_count;
static volatile uint8_t write_block_index;
static volatile uint16_t write_sample_index;
static volatile uint32_t block_sequence;
static volatile uint32_t dropped_block_count;
static volatile uint32_t pdm_error_count;
static volatile bool capture_paused;

static void app_pdm_pcm_reset_stream_state(void);
static bool app_pdm_pcm_claim_next_block_from_isr(void);
static bool app_pdm_pcm_push_free_block(uint8_t block_index);
static bool app_pdm_pcm_pop_free_block_from_isr(uint8_t *block_index);
static void app_pdm_pcm_publish_block_from_isr(BaseType_t *higher_priority_task_woken);
static void app_pdm_pcm_discard_fifo_from_isr(void);

/*******************************************************************************
* 函数名称: app_pdm_pcm_task_init
* 功能说明: 创建 PDM PCM 采集任务。
********************************************************************************/

cy_rslt_t app_pdm_pcm_task_init(void)
{
    BaseType_t ret;

    /* 队列长度和 block 数量一致。
     * 每个完整 block 都可以对应一个待处理消息，ISR 则等待消费者释放 block。
     */
    if (NULL == pdm_pcm_block_queue)
    {
        pdm_pcm_block_queue = xQueueCreate(APP_PDM_PCM_BLOCK_COUNT,
                                           sizeof(app_pdm_pcm_block_t));
        if (NULL == pdm_pcm_block_queue)
        {
            return CY_RSLT_TYPE_ERROR;
        }
    }

    ret = xTaskCreate(app_pdm_pcm_task,
                      "app_pdm_pcm_task",
                      APP_PDM_PCM_TASK_STACK_SIZE,
                      NULL,
                      APP_PDM_PCM_TASK_PRIORITY,
                      &pdm_pcm_task_handle);
    if (pdPASS != ret)
    {
        return CY_RSLT_TYPE_ERROR;
    }
    return CY_RSLT_SUCCESS;
}

QueueHandle_t app_pdm_pcm_get_queue(void)
{
    return pdm_pcm_block_queue;
}

bool app_pdm_pcm_receive_block(app_pdm_pcm_block_t *block, TickType_t ticks_to_wait)
{
    /* 推理任务从这里阻塞等待完整 10 ms block。
     * 这里拿到的是描述符，PCM 数据仍留在 recorded_data 对应行中。
     */
    if ((NULL == pdm_pcm_block_queue) || (NULL == block))
    {
        return false;
    }

    return (pdPASS == xQueueReceive(pdm_pcm_block_queue, block, ticks_to_wait));
}

void app_pdm_pcm_release_block(uint8_t block_index)
{
    if (APP_PDM_PCM_BLOCK_COUNT <= block_index)
    {
        return;
    }

    /* 推理任务复制或消费完该 block 后调用本函数。
     * 在释放之前，ISR 会认为该 block 不可用，不会覆盖它。
     */
    taskENTER_CRITICAL();
    if (APP_PDM_PCM_BLOCK_READY == block_state[block_index])
    {
        block_state[block_index] = APP_PDM_PCM_BLOCK_FREE;
        (void)app_pdm_pcm_push_free_block(block_index);
    }
    taskEXIT_CRITICAL();
}

uint32_t app_pdm_pcm_get_dropped_count(void)
{
    return dropped_block_count;
}

static void app_pdm_pcm_reset_stream_state(void)
{
    /* 重新启动采集流时，把状态机恢复到“block0 正在写，其余 block 空闲”。
     * 注意：队列也要清空，否则消费者可能拿到上一次运行残留的描述符。
     */
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < APP_PDM_PCM_BLOCK_COUNT; i++)
    {
        block_state[i] = APP_PDM_PCM_BLOCK_FREE;
    }

    write_block_index = 0;
    write_sample_index = 0;
    free_block_count = 0;
    block_sequence = 0;
    dropped_block_count = 0;
    pdm_error_count = 0;
    capture_paused = false;

    /* 启动 PDM 采集前，先预留第一个 block 给 ISR 写入。 */
    block_state[write_block_index] = APP_PDM_PCM_BLOCK_FILLING;
    audio_data_ptr = &recorded_data[write_block_index][write_sample_index];
    recorded_data_size = 0;

    for (uint8_t i = APP_PDM_PCM_BLOCK_COUNT; i > 1u; i--)
    {
        /* 栈是后进先出，这里倒序压入，第一次弹出时会优先得到 block1。 */
        (void)app_pdm_pcm_push_free_block((uint8_t)(i - 1u));
    }

    if (NULL != pdm_pcm_block_queue)
    {
        (void)xQueueReset(pdm_pcm_block_queue);
    }
    taskEXIT_CRITICAL();
}

static bool app_pdm_pcm_claim_next_block_from_isr(void)
{
    /* 从空闲块池申请下一个可写 block。
     * 成功时 ISR 继续无缝写入；失败时说明所有 block 都在队列中或被消费者占用。
     */
    uint8_t next_index;

    if (app_pdm_pcm_pop_free_block_from_isr(&next_index))
    {
        write_block_index = next_index;
        write_sample_index = 0;
        block_state[write_block_index] = APP_PDM_PCM_BLOCK_FILLING;
        audio_data_ptr = &recorded_data[write_block_index][write_sample_index];
        recorded_data_size = 0;
        capture_paused = false;
        return true;
    }

    /* 没有可复用 block。
     * 在消费者释放至少一个 block 前，继续清空硬件 FIFO 并丢弃数据，
     * 否则 PDM FIFO 可能溢出并持续触发中断。
     */
    capture_paused = true;
    audio_data_ptr = NULL;
    return false;
}

static bool app_pdm_pcm_push_free_block(uint8_t block_index)
{
    /* 把消费者已经处理完的 block 放回空闲池。
     * 该函数会在临界区或 ISR 上下文中被调用，所以内部只做常数时间操作。
     */
    if ((APP_PDM_PCM_BLOCK_COUNT <= block_index) ||
        (APP_PDM_PCM_BLOCK_COUNT <= free_block_count))
    {
        return false;
    }

    free_block_stack[free_block_count] = block_index;
    free_block_count++;
    return true;
}

static bool app_pdm_pcm_pop_free_block_from_isr(uint8_t *block_index)
{
    /* ISR 申请空闲 block，失败时上层会进入 capture_paused 并丢弃 FIFO 数据。 */
    if ((NULL == block_index) || (0u == free_block_count))
    {
        return false;
    }

    free_block_count--;
    *block_index = free_block_stack[free_block_count];
    return true;
}

static void app_pdm_pcm_publish_block_from_isr(BaseType_t *higher_priority_task_woken)
{
    uint8_t completed_index = write_block_index;
    app_pdm_pcm_block_t block_msg = {
        .data = &recorded_data[completed_index][0],
        .sample_count = APP_PDM_PCM_BLOCK_SAMPLES,
        .samples_per_channel = APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK,
        .block_index = completed_index,
        .sequence = block_sequence++,
        .dropped_count = dropped_block_count
    };

    /* 这里只发布描述符：
     * PCM 数据仍保留在 recorded_data 中，直到 app_pdm_pcm_release_block()
     * 将该 block 归还到空闲池。
     * 这样 ISR 中的工作量小，也避免每 10 ms 复制一整块 PCM 数据。
     */
    if ((NULL != pdm_pcm_block_queue) &&
        (pdPASS == xQueueSendFromISR(pdm_pcm_block_queue,
                                     &block_msg,
                                     higher_priority_task_woken)))
    {
        block_state[completed_index] = APP_PDM_PCM_BLOCK_READY;
    }
    else
    {
        /* 队列满或尚未创建时，不能把该 block 留在 READY 状态，
         * 否则空闲池会越来越少。这里直接丢弃该 block 并归还。
         */
        block_state[completed_index] = APP_PDM_PCM_BLOCK_FREE;
        (void)app_pdm_pcm_push_free_block(completed_index);
        dropped_block_count++;
    }

    if (!app_pdm_pcm_claim_next_block_from_isr())
    {
        /* 没有新的可写 block，说明消费者处理速度低于采集速度。
         * 后续 ISR 会清空硬件 FIFO 来保护系统实时性。
         */
        dropped_block_count++;
    }
}

static void app_pdm_pcm_discard_fifo_from_isr(void)
{
    /* 背压保护：如果所有 block 仍被消费者占用，
     * 丢弃当前 FIFO 数据片段，但继续服务中断源，避免硬件 FIFO 卡住。
     */
    for (uint8_t i = 0; i < RX_FIFO_TRIG_LEVEL; i++)
    {
        (void)Cy_PDM_PCM_Channel_ReadFifo(PDM0, LEFT_CH_INDEX);
        (void)Cy_PDM_PCM_Channel_ReadFifo(PDM0, RIGHT_CH_INDEX);
    }
}

/*******************************************************************************
* 函数名称: app_pdm_pcm_init
********************************************************************************
* 功能说明: 初始化 PDM PCM 硬件模块。
*
* 参数:
*  无
*
* 返回:
*  无
*
*******************************************************************************/
void app_pdm_pcm_init(void)
{
    cy_en_pdm_pcm_gain_sel_t gain_scale = CY_PDM_PCM_SEL_GAIN_NEGATIVE_37DB;
    
    /* 初始化 PDM PCM 硬件模块，底层配置来自 BSP 生成的 CYBSP_PDM_config。 */
    if(CY_PDM_PCM_SUCCESS != Cy_PDM_PCM_Init(PDM0, &CYBSP_PDM_config))
    {
        CY_ASSERT(0);
    }

    /* 先使能并初始化 PDM/PCM 左右声道。
     * Enable/Init 只完成硬件准备，真正开始采集放在 app_pdm_pcm_activate()。
     */
    Cy_PDM_PCM_Channel_Enable(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Enable(PDM0, RIGHT_CH_INDEX);

    Cy_PDM_PCM_Channel_Init(PDM0, &LEFT_CH_CONFIG, (uint8_t)LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM0, &RIGHT_CH_CONFIG, (uint8_t)RIGHT_CH_INDEX);
    
    /* 设置左右声道增益，保证两个麦克风通道幅值标尺一致。 */
    
    gain_scale = convert_db_to_pdm_scale((double)PDM_MIC_GAIN_VALUE);
    set_pdm_pcm_gain(gain_scale);
        
    /* 当前使用右声道中断作为触发源。
     * 右声道 FIFO 到半满阈值时，左声道通常也已经有同样数量的数据，
     * 因此 ISR 中按“先左后右”的顺序同步读出一组双声道样本。
     */
    Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);      
    Cy_PDM_PCM_Channel_SetInterruptMask(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);

    /* 注册并使能 PDM/PCM 硬件中断处理函数。 */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&PDM_IRQ_cfg, &pdm_interrupt_handler))
    {
        CY_ASSERT(0);
    }
    NVIC_ClearPendingIRQ(PDM_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(PDM_IRQ_cfg.intrSrc);
}

/*******************************************************************************
 * 函数名称: app_pdm_pcm_activate
 ********************************************************************************
* 功能说明: 激活左右声道，开始 PDM 到 PCM 的采集转换。
*
* 参数:
*  无
*
* 返回:
*  无
*
*******************************************************************************/
void app_pdm_pcm_activate(void)
{
    /* 初始化完成后激活左右通道采集 */
    Cy_PDM_PCM_Activate_Channel(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Activate_Channel(PDM0, RIGHT_CH_INDEX);
}

/*******************************************************************************
 * 函数名称: convert_db_to_pdm_scale
 ********************************************************************************
 * 功能说明:
 * 将 dB 增益值转换为 PDM scale 枚举值。
 * 参数:
 *  db  : dB 增益值
 * 返回:
 *  PDM scale 枚举值
 *
 *******************************************************************************/

cy_en_pdm_pcm_gain_sel_t convert_db_to_pdm_scale(double db)
{
    if (db<=PDM_PCM_MIN_GAIN)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_103DB; 
    }
    else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_103DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_97DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_97DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_97DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_91DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_91DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_91DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_85DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_85DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_85DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_79DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_79DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_79DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_73DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_73DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_73DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_67DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_67DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_67DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_61DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_61DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_61DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_55DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_55DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_55DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_49DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_49DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_49DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_43DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_43DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_43DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_37DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_37DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_37DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_31DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_31DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_31DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_25DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_25DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_25DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_19DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_19DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_19DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_13DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_13DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_13DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_7DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_7DB;
    }
    else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_7DB && db<=PDM_PCM_SEL_GAIN_NEGATIVE_1DB)
    {
        return CY_PDM_PCM_SEL_GAIN_NEGATIVE_1DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_NEGATIVE_1DB && db<=PDM_PCM_SEL_GAIN_5DB)
    {
        return CY_PDM_PCM_SEL_GAIN_5DB;
    }
     else if (db>PDM_PCM_SEL_GAIN_5DB && db<=PDM_PCM_SEL_GAIN_11DB)
    {
        return CY_PDM_PCM_SEL_GAIN_11DB;
    }
    else if (db>PDM_PCM_SEL_GAIN_11DB && db<=PDM_PCM_SEL_GAIN_17DB)
    {
        return CY_PDM_PCM_SEL_GAIN_17DB;
    }     
    else if (db>PDM_PCM_SEL_GAIN_17DB && db<=PDM_PCM_SEL_GAIN_23DB)
    {
        return CY_PDM_PCM_SEL_GAIN_23DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_23DB && db<=PDM_PCM_SEL_GAIN_29DB)
    {
        return CY_PDM_PCM_SEL_GAIN_29DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_29DB && db<=PDM_PCM_SEL_GAIN_35DB)
    {
        return CY_PDM_PCM_SEL_GAIN_35DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_35DB && db<=PDM_PCM_SEL_GAIN_41DB)
    {
        return CY_PDM_PCM_SEL_GAIN_41DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_41DB && db<=PDM_PCM_SEL_GAIN_47DB)
    {
        return CY_PDM_PCM_SEL_GAIN_47DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_47DB && db<=PDM_PCM_SEL_GAIN_53DB)
    {
        return CY_PDM_PCM_SEL_GAIN_53DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_53DB && db<=PDM_PCM_SEL_GAIN_59DB)
    {
        return CY_PDM_PCM_SEL_GAIN_59DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_59DB && db<=PDM_PCM_SEL_GAIN_65DB)
    {
        return CY_PDM_PCM_SEL_GAIN_65DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_65DB && db<=PDM_PCM_SEL_GAIN_71DB)
    {
        return CY_PDM_PCM_SEL_GAIN_71DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_71DB && db<=PDM_PCM_SEL_GAIN_77DB)
    {
        return CY_PDM_PCM_SEL_GAIN_77DB;
    } 
    else if (db>PDM_PCM_SEL_GAIN_77DB && db<=PDM_PCM_SEL_GAIN_83DB)
    {
        return CY_PDM_PCM_SEL_GAIN_83DB;
    } 
    else if (db>PDM_PCM_MAX_GAIN)
    {
        return CY_PDM_PCM_SEL_GAIN_83DB;
    }
    else 
    {
        return (cy_en_pdm_pcm_gain_sel_t) PDM_MIC_GAIN_VALUE;
    } 
    
}
/*******************************************************************************
 * 函数名称: set_pdm_pcm_gain
 ********************************************************************************
 * 
 * 设置左右声道的 PDM 增益 scale。
 *
 *******************************************************************************/
void set_pdm_pcm_gain(cy_en_pdm_pcm_gain_sel_t gain)
{

    Cy_PDM_PCM_SetGain(PDM0, RIGHT_CH_INDEX, gain);
    Cy_PDM_PCM_SetGain(PDM0, LEFT_CH_INDEX, gain);

}

/*******************************************************************************
* 函数名称: pdm_interrupt_handler
********************************************************************************
* 功能说明: 
*  PDM RX 中断处理函数。
*  每次中断读取每声道 RX_FIFO_TRIG_LEVEL 个采样点。16 kHz 下这相当于
*  2 ms 双声道 PCM，因此累计 5 次 ISR 数据后形成一个 10 ms 推理块。
*
*******************************************************************************/
void pdm_interrupt_handler(void)
{
    volatile uint32_t int_stat;
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* 只读取 masked status，避免处理没有打开掩码的中断源。 */
    int_stat = Cy_PDM_PCM_Channel_GetInterruptStatusMasked(PDM0, RIGHT_CH_INDEX);
    if(CY_PDM_PCM_INTR_RX_TRIGGER & int_stat)
    {
        /* 如果消费者处理变慢，先尝试在 block 被释放后恢复采集。
         * 如果仍没有空闲 block，则清空并丢弃当前 FIFO 数据片段。
         */
        if (capture_paused && !app_pdm_pcm_claim_next_block_from_isr())
        {
            dropped_block_count++;
            app_pdm_pcm_discard_fifo_from_isr();
            Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX,
                                              CY_PDM_PCM_INTR_RX_TRIGGER);
        }
        else
        {
            for(uint8_t i=0; i < RX_FIFO_TRIG_LEVEL; i++)
            {
                /* 双声道 PCM 按 L,R,L,R... 顺序保存。
                 * 推理侧可以直接使用双声道，也可以在拼窗口时下混成单声道。
                 * 当前阈值为 32，所以一次 ISR 写入 32 组 L/R 样本，即 64 个 int16_t。
                 */
                int32_t data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, LEFT_CH_INDEX);
                recorded_data[write_block_index][write_sample_index++] = (int16_t)(data);
                data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(PDM0, RIGHT_CH_INDEX);
                recorded_data[write_block_index][write_sample_index++] = (int16_t)(data);

                /* 一个完整 10 ms block 已经准备好。
                 * 将描述符放入队列，并把 ISR 写指针切到下一个空闲 block。
                 */
                if (APP_PDM_PCM_BLOCK_SAMPLES <= write_sample_index)
                {
                    app_pdm_pcm_publish_block_from_isr(&higher_priority_task_woken);
                    if (capture_paused)
                    {
                        break;
                    }
                }
            }

            /* 旧调试指针只在本次 FIFO 片段处理完后更新一次，
             * 避免在每个采样点上更新全局变量，减少 ISR 内部开销。
             */
            audio_data_ptr = capture_paused ?
                             NULL :
                             &recorded_data[write_block_index][write_sample_index];
            recorded_data_size = write_sample_index;

            Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX,
                                              CY_PDM_PCM_INTR_RX_TRIGGER);
        }
    }
    if((CY_PDM_PCM_INTR_RX_FIR_OVERFLOW | CY_PDM_PCM_INTR_RX_OVERFLOW|
    CY_PDM_PCM_INTR_RX_IF_OVERFLOW | CY_PDM_PCM_INTR_RX_UNDERFLOW) & int_stat)
    {
        /* 保留一个简单错误计数，便于后续诊断；
         * 随后清除硬件错误标志，让采集可以继续运行。
         */
        pdm_error_count++;
        Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);
    }

    /* 如果入队操作唤醒了更高优先级的推理任务，
     * 则在退出 ISR 后立即切换到该任务。
     */
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/*******************************************************************************
* 函数名称: app_pdm_pcm_deactivate
********************************************************************************
* 功能说明: 关闭左右声道采集。
*
* 参数:
*  无
*
* 返回:
*  无
*
*******************************************************************************/
void app_pdm_pcm_deactivate(void)
{
    Cy_PDM_PCM_DeActivate_Channel(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_DeActivate_Channel(PDM0, RIGHT_CH_INDEX);
}

void app_pdm_pcm_task(void *pvParameters)
{
    (void) pvParameters;

    /* PDM 硬件启动由该任务负责：
     * 1. 初始化硬件和中断；
     * 2. 重置 block 状态机和队列；
     * 3. 激活左右声道，让后续采集完全由 ISR 驱动。
     *
     * 把硬件初始化放在任务上下文中，可以避免队列未准备好时就启动中断。
     */
    app_pdm_pcm_init();
    app_pdm_pcm_reset_stream_state();
    app_pdm_pcm_activate();

    for(;;)
    {
        /* 激活后采集流程由中断驱动。
         * 任务保持存活，后续可以在这里扩展暂停/恢复或诊断上报逻辑。
         */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
