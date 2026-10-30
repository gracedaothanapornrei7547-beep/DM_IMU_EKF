#include "BMI088Middleware.h"
#include "main.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/**
************************************************************************
* @brief:      	BMI088_GPIO_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088������GPIO��ʼ������
************************************************************************
**/
void BMI088_GPIO_init(void)
{

}
/**
************************************************************************
* @brief:      	BMI088_com_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088������ͨ�ų�ʼ������
************************************************************************
**/
void BMI088_com_init(void)
{


}
/**
************************************************************************
* @brief:      	BMI088_delay_ms(uint16_t ms)
* @param:       ms - Ҫ�ӳٵĺ�����
* @retval:     	void
* @details:    	�ӳ�ָ���������ĺ���������΢���ӳ�ʵ��
************************************************************************
**/
void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}
/**
************************************************************************
* @brief:      	BMI088_delay_us(uint16_t us)
* @param:       us - Ҫ�ӳٵ�΢����
* @retval:     	void
* @details:    	΢�뼶�ӳٺ�����ʹ��SysTick��ʱ��ʵ��
************************************************************************
**/
void BMI088_delay_us(uint16_t us)
{
    /* 用 DWT 周期计数器计时(独立外设,不依赖 SysTick)。
       原版读 SysTick->VAL:FreeRTOS 下 SysTick 在 osKernelStart 之前是停的(LOAD=0),
       而本函数在调度器启动前(IMU 初始化阶段)就被调用 -> 会死等。改用 DWT。
       要求调用前 DWT_Init() 已执行(见 imu.c 的 IMU_Init 顺序)。 */
    uint32_t ticks = us * (SystemCoreClock / 1000000U);   /* 240MHz -> us*240 cycle */
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088���ٶȼ�Ƭѡ�ź��õͣ�ʹ�䴦��ѡ��״̬
************************************************************************
**/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088���ٶȼ�Ƭѡ�ź��øߣ�ʹ�䴦�ڷ�ѡ��״̬
************************************************************************
**/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088������Ƭѡ�ź��õͣ�ʹ�䴦��ѡ��״̬
************************************************************************
**/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088������Ƭѡ�ź��øߣ�ʹ�䴦�ڷ�ѡ��״̬
************************************************************************
**/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_read_write_byte(uint8_t txdata)
* @param:       txdata - Ҫ���͵�����
* @retval:     	uint8_t - ���յ�������
* @details:    	ͨ��BMI088ʹ�õ�SPI���߽��е��ֽڵĶ�д����
************************************************************************
**/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/**
************************************************************************
* @brief:       BMI088_read_muli(uint8_t *rx, uint16_t len)
* @param:       rx  - 接收缓冲区; len - 字节数(BMI088 单次 ≤8)
* @retval:      void
* @details:     CS 已拉低的事务内一次性块读 len 字节。读阶段 MOSI 为
*               don't-care,发固定 0x55 dummy。替逐字节循环,省每字节
*               HAL_SPI 调用开销(Step③ 优化)。
************************************************************************
**/
void BMI088_read_muli(uint8_t *rx, uint16_t len)
{
    static const uint8_t dummy[16] = {
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    };
    if (len > sizeof(dummy)) len = sizeof(dummy);   /* 防越界:BMI088 单次读 ≤8,16 足够 */
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, (uint8_t *)dummy, rx, len, 1000);
}

