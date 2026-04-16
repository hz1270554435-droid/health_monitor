#include "APP_UART_RADAR.h"

static enum {waiting_SOF ,
            state_SOF ,
            state_ID ,
            state_LEN ,
            state_TYPE ,
            state_SOF ,
            state_Head_CK ,
            state_DATA ,
            state_Data_CK}RECIEVE_FSM_STATE;

cy_rslt_t APP_UART_RADAR_Init(void)
{

    return CY_RSLT_SUCCESS;
}

static uint32_t read_uart_data()
{

}

cy_rslt_t APP_UART_RADAR_GetData(uint32_t* Reciece_buffer)
{
    Reciece_buffer = read_uart_data();
    return CY_RSLT_SUCCESS;
}