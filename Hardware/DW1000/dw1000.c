#include "dw1000.h"

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO used for DW1000 reset
	// done by main.c(MX_GPIO_Init) 

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
	
	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = DW1000_RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(DW1000_RST_PORT, &GPIO_InitStruct);

  HAL_Delay(2);
}

int spi_set_rate_low(void)
{
		if (HAL_SPI_DeInit(&DW1000_SPI_Handle) != HAL_OK)
		{
			return -1;
		}
	
		hspi1.Instance = SPI1;
		hspi1.Init.Mode = SPI_MODE_MASTER;
		hspi1.Init.Direction = SPI_DIRECTION_2LINES;
		hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi1.Init.NSS = SPI_NSS_SOFT;
		hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
		hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi1.Init.CRCPolynomial = 7;
		hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
		if (HAL_SPI_Init(&DW1000_SPI_Handle) != HAL_OK)
		{
			return -1;
		}
		
		return 0;
}

int spi_set_rate_high(void)
{
		if (HAL_SPI_DeInit(&DW1000_SPI_Handle) != HAL_OK)
		{
			return -1;
		}
	
		hspi1.Instance = SPI1;
		hspi1.Init.Mode = SPI_MODE_MASTER;
		hspi1.Init.Direction = SPI_DIRECTION_2LINES;
		hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi1.Init.NSS = SPI_NSS_SOFT;
		hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi1.Init.CRCPolynomial = 7;
		hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
		if (HAL_SPI_Init(&DW1000_SPI_Handle) != HAL_OK)
		{
			return -1;
		}
		
		return 0;
}
