/*
 * dshot.h
 *
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 *
 */


#include "dshot.h"


/* Variables */
static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];


/* Static functions */
// dshot init
static uint32_t dshot_choose_type(dshot_type_e dshot_type);
static void dshot_set_timer(dshot_type_e dshot_type, Motor* motor);
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma);
static void dshot_put_tc_callback_function(Motor* motor);
static void dshot_start_pwm(Motor* motor);

// dshot write
static uint16_t dshot_prepare_packet(uint16_t value);
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value);
static void dshot_prepare_dmabuffer_all(uint16_t motor_value);
static void dshot_dma_start(Motor* motor);
static void dshot_enable_dma_request(Motor* motor);


/* Functions */
void dshot_init(dshot_type_e dshot_type, Motor* motor)
{
	dshot_set_timer(dshot_type, motor);
	dshot_put_tc_callback_function(motor);
	dshot_start_pwm(motor);
}

void dshot_write(uint16_t motor_value, Motor* motor)
{
	dshot_prepare_dmabuffer_all(motor_value);
	dshot_dma_start(motor);
	dshot_enable_dma_request(motor);
}


/* Static functions */
static uint32_t dshot_choose_type(dshot_type_e dshot_type)
{
	switch (dshot_type)
	{
		case(DSHOT600):
				return DSHOT600_HZ;

		case(DSHOT300):
				return DSHOT300_HZ;

		default:
		case(DSHOT150):
				return DSHOT150_HZ;
	}
}

static void dshot_set_timer(dshot_type_e dshot_type, Motor *motor)
{
	uint16_t dshot_prescaler;
	uint32_t timer_clock = TIMER_CLOCK; // all timer clock is same as SystemCoreClock in stm32f411

	// Calculate prescaler by dshot type
	dshot_prescaler = lrintf((float) timer_clock / dshot_choose_type(dshot_type) + 0.01f) - 1;

	// motor1
	__HAL_TIM_SET_PRESCALER(motor->htim, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(motor->htim, MOTOR_BITLENGTH);
}

// __HAL_TIM_DISABLE_DMA is needed to eliminate the delay between different dshot signals
// I don't know why :(
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}

static void dshot_put_tc_callback_function(Motor* motor)
{
	// TIM_DMA_ID_CCx depends on timer channel
	motor->htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
}

static void dshot_start_pwm(Motor* motor)
{
	// Start the timer channel now.
    // Enabling/disabling DMA request can restart a new cycle without PWM start/stop.
  	HAL_TIM_PWM_Start(motor->htim, motor->Channel);
}

static uint16_t dshot_prepare_packet(uint16_t value)
{
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

// Convert 16 bits packet to 16 pwm signal
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value)
{
	uint16_t packet;
	packet = dshot_prepare_packet(value);

	for(int i = 0; i < 16; i++)
	{
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
}

static void dshot_prepare_dmabuffer_all(uint16_t motor_value)
{
	dshot_prepare_dmabuffer(motor1_dmabuffer, motor_value);
}

static void dshot_dma_start(Motor* motor)
{
	HAL_DMA_Start_IT(motor->htim->hdma[TIM_DMA_ID_CC4], (uint32_t)motor1_dmabuffer, (uint32_t)&motor->htim->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);
}

static void dshot_enable_dma_request(Motor* motor)
{
	__HAL_TIM_ENABLE_DMA(motor->htim, TIM_DMA_CC4);
}
