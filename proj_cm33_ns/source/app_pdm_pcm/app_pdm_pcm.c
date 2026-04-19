/******************************************************************************
* File Name : app_pdm_pcm.c
*
* Description : PDM 转 PCM 采集模块，将音频整理为 10 ms 环形缓冲块供推理任务消费。
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
* 全局变量
*******************************************************************************/
/* PDM/PCM 中断配置参数 */
const cy_stc_sysint_t PDM_IRQ_cfg = {
    .intrSrc = (IRQn_Type)PDM_IRQ,
    .intrPriority = PDM_PCM_ISR_PRIORITY
};

typedef enum
{
    /* 该块空闲，ISR 可以重新写入。 */
    APP_PDM_PCM_BLOCK_FREE = 0,
    /* ISR 当前正在向该块追加 PCM 采样。 */
    APP_PDM_PCM_BLOCK_FILLING,
    /* 该块已经入队给消费者，释放前不能被覆盖。 */
    APP_PDM_PCM_BLOCK_READY
} app_pdm_pcm_block_state_t;

/* 面向推理任务的 10 ms 双声道音频块环形缓冲区。
 * 该缓冲区放在共享 SoC 内存中，后续如果把推理放到 CM55，
 * 可以直接消费同一批 PCM 块，避免从 CM33 本地 SRAM 再复制一份。
 */
int16_t recorded_data[APP_PDM_PCM_BLOCK_COUNT][APP_PDM_PCM_BLOCK_SAMPLES]
    __attribute__((section(".cy_shared_socmem"))) = {0};

int32_t recorded_data_size;
volatile int16_t *audio_data_ptr = NULL;

/* 队列只传递很小的块描述符，不复制原始 PCM 数据。 */
static QueueHandle_t pdm_pcm_block_queue = NULL;
static TaskHandle_t pdm_pcm_task_handle = NULL;

/* ISR 侧生产者状态。
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
    /* 从当前写入位置向前查找已经被消费者释放的 block。
     * 这样即使推理任务短时间慢于采集，也不会覆盖已经入队的数据。
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

    /* 这里只发布描述符。
     * PCM 数据仍保留在 recorded_data 中，直到 app_pdm_pcm_release_block()
     * 将该 block 归还到空闲池。
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
        block_state[completed_index] = APP_PDM_PCM_BLOCK_FREE;
        (void)app_pdm_pcm_push_free_block(completed_index);
        dropped_block_count++;
    }

    if (!app_pdm_pcm_claim_next_block_from_isr())
    {
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
    
    /* 初始化 PDM PCM 硬件模块 */
    if(CY_PDM_PCM_SUCCESS != Cy_PDM_PCM_Init(PDM0, &CYBSP_PDM_config))
    {
        CY_ASSERT(0);
    }

    /* 初始化 PDM/PCM 左右声道 */
    /* 先使能 PDM 通道，后续开始录音时再激活通道 */
    Cy_PDM_PCM_Channel_Enable(PDM0, LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Enable(PDM0, RIGHT_CH_INDEX);

    Cy_PDM_PCM_Channel_Init(PDM0, &LEFT_CH_CONFIG, (uint8_t)LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM0, &RIGHT_CH_CONFIG, (uint8_t)RIGHT_CH_INDEX);
    
    /* 设置左右声道增益 */
    
    gain_scale = convert_db_to_pdm_scale((double)PDM_MIC_GAIN_VALUE);
    set_pdm_pcm_gain(gain_scale);
        
    /* 当前使用右声道中断作为触发源，先清标志再打开中断掩码 */
    Cy_PDM_PCM_Channel_ClearInterrupt(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);      
    Cy_PDM_PCM_Channel_SetInterruptMask(PDM0, RIGHT_CH_INDEX, CY_PDM_PCM_INTR_MASK);

    /* 注册 PDM/PCM 硬件中断处理函数 */
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

    /* PDM 硬件启动由该任务负责。
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
