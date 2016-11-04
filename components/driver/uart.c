// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>

#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"

#include "rom/ets_sys.h"

#include "soc/uart_struct.h"
#include "uart.h"

#define UART_DBG_ENABLE    (0)
#define UART_WARING_ENABLE (0)
#define UART_ERROR_ENABLE  (0)
#define UART_INFO_ENABLE   (0)


//DBG INFOR
#if UART_INFO_ENABLE
    #define UART_INFO ets_printf
#else
    #define UART_INFO(...)
#endif
#if UART_WARING_ENABLE
    #define UART_WARING(format,...) do{\
		    ets_printf("[waring][%s#%u]",__FUNCTION__,__LINE__);\
		    ets_printf(format,##__VA_ARGS__);\
	    }while(0)
#else
     #define UART_WARING(...)
#endif
#if UART_ERROR_ENABLE
    #define UART_ERROR(format,...) do{\
		ets_printf("[error][%s#%u]",__FUNCTION__,__LINE__);\
		ets_printf(format,##__VA_ARGS__);\
	}while(0)
#else
    #define UART_ERROR(...)
#endif



#define DBG_LINES() do{\
	    ets_printf("[%s##%u]\n",__FUNCTION__,__LINE__);\
	    ets_delay_us(100*1000);\
    }while(0)
#define DBG_PARAM(format,...) do{\
    	 ets_printf("[%s##%u]\n",__FUNCTION__,__LINE__);\
        ets_printf(format,##__VA_ARGS__);\
        ets_delay_us(100*1000);\
    }while(0)
enum {
    UART_EVENT_RX_CHAR,
    UART_EVENT_MAX
};

uart_event_callback global_event_p=NULL;
LOCAL STATUS uart_tx_a_char(uint8_t uart, uint8_t TxChar)
{
	uart_dev_t* UARTx = UART(uart);
	uint16_t safe_cir=0;
    while (true) {

    	uint8_t tx_fifo_cnt = UARTx->status.txfifo_cnt;
		safe_cir++;
        if (tx_fifo_cnt < 126) {
            break;
        }
		if(safe_cir>100){
          goto __ERR__;
		}
    }

    UARTx->fifo.rw_byte = TxChar & 0xff;
    return OK;
__ERR__:
	 return FAIL;
}

LOCAL void uart1_write_char(char c)
{
    if (c == '\n') {
        uart_tx_a_char(UART1, '\r');
        uart_tx_a_char(UART1, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_a_char(UART1, c);
    }
}

LOCAL void uart0_write_char(char c)
{
    if (c == '\n') {
        uart_tx_a_char(UART0, '\r');
        uart_tx_a_char(UART0, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_a_char(UART0, c);
    }
}

uint32_t uart_tx_chars(UART_Port uart_no,char* buffer,uint32_t len)
{
    uint8_t i=0;
    uart_dev_t* UARTx = UART(uart_no);
	uint8_t tx_fifo_cnt = UARTx->status.txfifo_cnt;
	uint8_t tx_remaind_fifo_cnt=(UART_FIFO_CHAR_CNT-tx_fifo_cnt);
    uint8_t copy_cnt=0;
	copy_cnt=(len>=tx_remaind_fifo_cnt?tx_remaind_fifo_cnt:len);
	for(i=0;i<copy_cnt;i++){
       UARTx->fifo.rw_byte = buffer[i];
	}
	return i;
}

void UART_SetWordLength(UART_Port uart_no, UART_WordLength len)
{
    uart_dev_t* UARTx = UART(uart_no);
    UARTx->conf0.bit_num = len;
}

void
UART_SetStopBits(UART_Port uart_no, UART_StopBits bit_num)
{
	uart_dev_t* UARTx = UART(uart_no);
	UARTx->conf0.stop_bit_num = bit_num;
}

void UART_SetLineInverse(UART_Port uart_no, UART_LineLevelInverse inverse_mask)
{
    CLEAR_PERI_REG_MASK(UART_CONF0_REG(uart_no), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0_REG(uart_no), inverse_mask);
}

void UART_SetParity(UART_Port uart_no, UART_ParityMode Parity_mode)
{
	uart_dev_t* UARTx = UART(uart_no);
    UARTx->conf0.parity_en = 0;

    if (Parity_mode == USART_Parity_None) {
    } else {
        UARTx->conf0.parity = Parity_mode;
        UARTx->conf0.parity_en = 1;
    }
}

void UART_SetBaudrate(UART_Port uart_no, uint32_t baud_rate)
{
	uart_dev_t* UARTx = UART(uart_no);
	uint32_t clk_div = ((UART_CLK_FREQ<<4)/baud_rate);
	UARTx->clk_div.div_int = clk_div>>4 ;
	UARTx->clk_div.div_frag = clk_div & 0xf;
}
uint32_t UART_GetBaudrate(UART_Port uart_no)
{
	uart_dev_t* UARTx = UART(uart_no);
	uint32_t baudrate=0;
	uint32_t clk_div=UARTx->clk_div.div_int ;
    baudrate=UART_CLK_FREQ/clk_div;
	return baudrate;
}
//only when USART_HardwareFlowControl_RTS is set , will the rx_thresh value be set.
void UART_SetFlowCtrl(UART_Port uart_no, UART_HwFlowCtrl flow_ctrl, uint8_t rx_thresh)
{
	uart_dev_t* UARTx = UART(uart_no);

    if (flow_ctrl & USART_HardwareFlowControl_RTS) {
        UARTx->conf1.rx_flow_thrhd = rx_thresh;
        UARTx->conf1.rx_flow_en = 1;
    } else {
        UARTx->conf1.rx_flow_en = 0;
    }

    if (flow_ctrl & USART_HardwareFlowControl_CTS) {
        SET_PERI_REG_MASK(UART_CONF0_REG(uart_no), UART_TX_FLOW_EN);
        UARTx->conf0.tx_flow_en = 1;
    } else {
        UARTx->conf0.tx_flow_en = 0;
    }

}

void UART_WaitTxFifoEmpty(UART_Port uart_no) //do not use if tx flow control enabled
{
	uart_dev_t* UARTx = UART(uart_no);
    while(UARTx->status.txfifo_cnt);
    //There wll delay a byte
    //Beacause the Fifo cnt is zero not represent the data have sended.
    //Fifo cnt is zero represent the data move to Buffer,not
	uint32_t delay_us=0;
	uint32_t baud=UART_GetBaudrate(uart_no);
	//ets_printf("Baud Rate:%u\n",baud);
	uint8_t  byte_bits_on_uart=10*2;
	delay_us=byte_bits_on_uart*(1000000)/baud;//Because the baudre the time is second .change it to us.
	ets_delay_us(delay_us);
}

void UART_ResetFifo(UART_Port uart_no)
{
	uart_dev_t* UARTx = UART(uart_no);
    UARTx->conf0.rxfifo_rst = 1;
    UARTx->conf0.txfifo_rst = 1;
    UARTx->conf0.rxfifo_rst = 0;
    UARTx->conf0.txfifo_rst = 0;
}

void UART_ClearIntrStatus(UART_Port uart_no, uint32_t clr_mask)
{
    WRITE_PERI_REG(UART_INT_CLR_REG(uart_no), clr_mask);
}

void UART_SetIntrEna(UART_Port uart_no, uint32_t ena_mask)
{
    SET_PERI_REG_MASK(UART_INT_ENA_REG(uart_no), ena_mask);
}
void Uart_EnableRxIntr(UART_Port uart_no)
{
     SET_PERI_REG_MASK(UART_INT_ENA_REG(uart_no),UART_RXFIFO_FULL_INT_ENA|UART_RXFIFO_TOUT_INT_ENA );
}
void Uart_DisableRxIntr(UART_Port uart_no)
{
	 CLEAR_PERI_REG_MASK(UART_INT_ENA_REG(uart_no),UART_RXFIFO_FULL_INT_ENA|UART_RXFIFO_TOUT_INT_ENA );
}
void Uart_EnableTxIntr(UART_Port uart_no)
{
	SET_PERI_REG_MASK(UART_INT_ENA_REG(uart_no),UART_TXFIFO_EMPTY_INT_ENA);
}
void Uart_DisableTxIntr(UART_Port uart_no)
{
    CLEAR_PERI_REG_MASK(UART_INT_ENA_REG(uart_no),UART_TXFIFO_EMPTY_INT_ENA);
}
void UART_register_event_callback(uart_event_callback func)
{
    if(NULL==func){
      UART_ERROR("user set uart_event_callback in NULL\n");
	}
    global_event_p=func;
}

void  UART_intr_handler_register(void *fn, void *arg, uint32_t uart_id)
{
    uint32_t int_source;

    switch (uart_id) {
        case UART0:
            int_source = ETS_UART0_INTR_SOURCE;
            break;
        case UART1:
            int_source = ETS_UART1_INTR_SOURCE;
            break;
        case UART2:
        default:
            int_source = ETS_UART2_INTR_SOURCE;
            break;
    }

    intr_matrix_set(0, int_source, ETS_UART0_INUM);
    ESP_UART0_INTR_ATTACH(fn, arg);
}

void UART_SetPrintPort(UART_Port uart_no)
{
    if (uart_no == 1) {
		ets_install_putc1(uart1_write_char);
       }
	else {
       ets_install_putc1(uart0_write_char);

    }
}

void UART_InitConfig(UART_Port uart_no, UART_ConfigTypeDef *pUARTConfig)
{
    uart_dev_t* UARTx = UART(uart_no);

    UART_SetFlowCtrl(uart_no, pUARTConfig->flow_ctrl, pUARTConfig->UART_RxFlowThresh);
    UART_SetBaudrate(uart_no, pUARTConfig->baud_rate);

    UARTx->conf0.val = (((pUARTConfig->parity == USART_Parity_None) ? 0x0 : (UART_PARITY_EN | pUARTConfig->parity))
                           | (pUARTConfig->stop_bits << UART_STOP_BIT_NUM_S)
                           | (pUARTConfig->data_bits << UART_BIT_NUM_S)
                           | ((pUARTConfig->flow_ctrl & USART_HardwareFlowControl_CTS) ? UART_TX_FLOW_EN : 0x0)
                           | pUARTConfig->UART_InverseMask | UART_TICK_REF_ALWAYS_ON);
    UART_ResetFifo(uart_no);
}

void UART_IntrConfig(UART_Port uart_no,  UART_IntrConfTypeDef *pUARTIntrConf)
{
    uart_dev_t* UARTx = UART(uart_no);

    UART_ClearIntrStatus(uart_no, UART_INTR_MASK);

    if(pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_TOUT_INT_ENA){
        UARTx->conf1.rx_tout_thrhd = ((pUARTIntrConf->UART_RX_TimeOutIntrThresh)&UART_RX_TOUT_THRHD);
        UARTx->conf1.rx_tout_en = 1;
    }else{
    	UARTx->conf1.rx_tout_en = 0;
    }

    if(pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_FULL_INT_ENA){
    	UARTx->conf1.rxfifo_full_thrhd = pUARTIntrConf->UART_RX_FifoFullIntrThresh;
    }

    if(pUARTIntrConf->UART_IntrEnMask & UART_TXFIFO_EMPTY_INT_ENA){
    	UARTx->conf1.txfifo_empty_thrhd = pUARTIntrConf->UART_TX_FifoEmptyIntrThresh;
    }

    UARTx->int_ena.val = 0;
    UARTx->int_ena.val = pUARTIntrConf->UART_IntrEnMask;
	WRITE_PERI_REG(UART_INT_ENA_REG(uart_no),pUARTIntrConf->UART_IntrEnMask);
}
