/**
Note:
    -
    - Led PC13 as informing to users to know codo worked or not.
*/
#include "stm32f10x.h"
#include <stdio.h>

/* Connect to port A as A0, A1, A2, A3, A4*/
#define LINE_DETECTOR_SENSOR_OUT1  GPIO_Pin_0
#define LINE_DETECTOR_SENSOR_OUT2  GPIO_Pin_1
#define LINE_DETECTOR_SENSOR_OUT3  GPIO_Pin_2
#define LINE_DETECTOR_SENSOR_OUT4  GPIO_Pin_3
#define LINE_DETECTOR_SENSOR_OUT5  GPIO_Pin_4

#define ADC1_DR_ADDRESS        ((uint32_t)0x4001244C)
#define NUMBER_OF_ADC_CHANNEL   5U

#define FLASH_ADDRESS          0x8007C00U
#define AVERAGE_SENSOR_LINES   4000U
#define CHECK_BACKGROUND       2000U
#define AVERAGE_ADC            200U
#define BASE_SPEED             100

typedef enum
{
    ROBOT_ERRORS = 0U,    /* There is a error */
    ROBOT_NO_ERRORS      /* There is no errors */
} RobotStatusType;

typedef enum
{
    IN_LINE        = 1U,
    OUT_LINE 
} StatusOfEachLineType;

typedef enum
{
    FALSE = 0,
    TRUE = 1
} Boolean;

typedef struct
{
    Boolean WhileBackground;
    StatusOfEachLineType StatusOfEachLine[NUMBER_OF_ADC_CHANNEL];
} InfoRobotType;

void DMA_ConfigChannel_1(uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTranfer);
void ADC_Config(void);
void GPIO_Config(void);
void Flash_Unlock(void);
void Flash_Erase(volatile uint32_t u32StartAddr);
void Flash_Write(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length);
void Flash_Read(volatile uint32_t u32StartAddr, uint8_t* u8BufferRead, uint32_t u32Length);
RobotStatusType WriteSensorDataToFlash(uint16_t *u16AverageLines);
uint16_t AverageAdcValues(uint8_t u8Channel);
void GetInfoOfEachLines(uint16_t *u16AverageLines, InfoRobotType *InformationRobot);
void Delay(uint32_t u32Timeout);
Boolean CheckWhileBackground(uint16_t *pAdcValuesTemp);
void PD_Control(uint8_t Kp, uint8_t Kd);
int8_t GetErrorStatusOto(InfoRobotType *InformationRobot);
void SysTick_Interrupt(void);
void SysTick_Handler(void);
void PWM_Timer4_Muti_Channels(void);
void SpeedMotor(uint8_t SpeedMotor1, uint8_t SpeedMotor2, uint8_t SpeedMotor3, uint8_t SpeedMotor4);

static uint16_t u16AdcValues[NUMBER_OF_ADC_CHANNEL] = {0};
static RobotStatusType RobotStatus = ROBOT_ERRORS;
static InfoRobotType InfoRobot;
static Boolean ReadFlashOnce = TRUE;
static volatile int8_t s8Error = 0;
static volatile int8_t PriousError = 0;
static volatile uint8_t u8LeftSpeed = 0;
static volatile uint8_t u8RightSpeed = 0;
static uint16_t u16AverageSensorLine[NUMBER_OF_ADC_CHANNEL] = {0};

int main(void)
{
    GPIO_Config(); 
    /* Off led of Pin C13*/
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
		
	  PWM_Timer4_Muti_Channels();
		SysTick_Interrupt();
    ADC_Config();
    DMA_ConfigChannel_1((uint32_t *)ADC1_DR_ADDRESS, (uint32_t *)(uint32_t)u16AdcValues, NUMBER_OF_ADC_CHANNEL);
	  
    while(1)
    {
        RobotStatus = WriteSensorDataToFlash(&u16AverageSensorLine[0]);
        if (RobotStatus != ROBOT_NO_ERRORS)
        {
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            while(1);
        }
    }
}
void PD_Control(uint8_t Kp, uint8_t Kd)
{
    int16_t SpeedChange = 0U;
	
		SpeedChange = Kp*s8Error + Kd*(s8Error - PriousError);
	 
		if (SpeedChange > 100) 
		{
			SpeedChange = 100;
		}
		else if (SpeedChange < -100) 
		{
			SpeedChange = -100;
		}
		else
		{
			/* Do nothing */
		}
	
	  if (s8Error > 0)
		{
			u8LeftSpeed = (uint8_t)BASE_SPEED;
		  u8RightSpeed = (uint8_t)(BASE_SPEED - SpeedChange);
		}
		else if (s8Error < 0)
		{
			u8LeftSpeed = (uint8_t)(BASE_SPEED + SpeedChange);
			u8RightSpeed = (uint8_t)BASE_SPEED;
		}
		else
		{
			u8LeftSpeed = 100;
			u8RightSpeed = 100;
		}
	
	PriousError = s8Error;
}
void GetInfoOfEachLines(uint16_t *u16AverageLines, InfoRobotType *InformationRobot)
{
    uint8_t u8Count = 0;

    for (u8Count = 0; u8Count < 5U; u8Count++)
    {
			/* If it is while background */ 
			if (InfoRobot.WhileBackground == TRUE)
			{
        if (u16AverageLines[u8Count] < (AverageAdcValues(u8Count)))
        {
            InformationRobot->StatusOfEachLine[u8Count] = OUT_LINE;
        }
        else
        {
             InformationRobot->StatusOfEachLine[u8Count] = IN_LINE;
        }
			}
			/* If it is black background */ 
			else
			{
				if (u16AverageLines[u8Count] > (AverageAdcValues(u8Count)))
        {
             InformationRobot->StatusOfEachLine[u8Count] = OUT_LINE;
        }
        else
        {
            InformationRobot->StatusOfEachLine[u8Count] = IN_LINE;
        }
			}
    }
		/* Get Error of Oto */
		s8Error = GetErrorStatusOto(&InfoRobot);
}
/*
*  0 1 2 3 4
*
*/
int8_t GetErrorStatusOto(InfoRobotType *InformationRobot)
{
	int8_t TempErrors = 0;
	
	if((InformationRobot->StatusOfEachLine[0] == IN_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = -4;
	}
	else if((InformationRobot->StatusOfEachLine[0] == IN_LINE) && (InformationRobot->StatusOfEachLine[1] == IN_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = -3;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == IN_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = -2;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == IN_LINE) && (InformationRobot->StatusOfEachLine[2] == IN_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = -1;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == IN_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = 0;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == IN_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == IN_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = 1;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == IN_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		TempErrors = 2;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == IN_LINE) && (InformationRobot->StatusOfEachLine[4] == IN_LINE))
	{
		TempErrors = 3;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == IN_LINE))
	{
		TempErrors = 4;
	}
	else if((InformationRobot->StatusOfEachLine[0] == OUT_LINE) && (InformationRobot->StatusOfEachLine[1] == OUT_LINE) && (InformationRobot->StatusOfEachLine[2] == OUT_LINE) && \
		(InformationRobot->StatusOfEachLine[3] == OUT_LINE) && (InformationRobot->StatusOfEachLine[4] == OUT_LINE))
	{
		if (s8Error > 0)
		{
			TempErrors = 5;
		}
		else
		{
			TempErrors = -5;
		}
	}
	
	return TempErrors;
}
uint16_t AverageAdcValues(uint8_t u8Channel)
{
    uint32_t u32AverageAdcValues = 0U;
    uint32_t u8Count = 0;

    for (u8Count = 0; u8Count < AVERAGE_ADC; u8Count++)
    {
        u32AverageAdcValues = u32AverageAdcValues + u16AdcValues[u8Channel];
    }

    return (uint16_t)(u32AverageAdcValues/AVERAGE_ADC);
}
RobotStatusType WriteSensorDataToFlash(uint16_t *u16AverageLines)
{
    uint8_t u16ReadBuffer[NUMBER_OF_ADC_CHANNEL*2U];
    uint32_t u8Count = 0;
    uint16_t u16AdcValuesTemp[NUMBER_OF_ADC_CHANNEL] = {0};
    uint16_t u16AdcValuesMin[NUMBER_OF_ADC_CHANNEL] = {0};
    uint16_t u16AdcValuesMax[NUMBER_OF_ADC_CHANNEL] = {0};
    uint8_t u8CountChannel = 0u;
	Boolean WhileBackgroundTemp = FALSE;
		
    RobotStatus = ROBOT_NO_ERRORS;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0U)
    {
        Delay(100000);
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0U)
        {
            /* Erase sector */
            Flash_Erase(FLASH_ADDRESS);
            Flash_Read(FLASH_ADDRESS, (uint8_t *)u16ReadBuffer, NUMBER_OF_ADC_CHANNEL*2U);
            for (u8Count = 0; u8Count < (NUMBER_OF_ADC_CHANNEL*2U); u8Count++)
            {
                if(u16ReadBuffer[u8Count] != 0xFF)
                {
                    RobotStatus = ROBOT_ERRORS;
                    break;
                }
            }

            if (ROBOT_NO_ERRORS == RobotStatus)
            {
                /******************************** Average of each sesors *************************************/
                /* Turn off led of C13 pin */
                GPIO_ResetBits(GPIOC, GPIO_Pin_13);

                for (u8CountChannel = 0; u8CountChannel < NUMBER_OF_ADC_CHANNEL; u8CountChannel++)
                {
                    u16AdcValuesMin[u8CountChannel] = AverageAdcValues(u8CountChannel);
                }

                for (u8Count = 0; u8Count < AVERAGE_SENSOR_LINES; u8Count++)
                {
                    for (u8CountChannel = 0; u8CountChannel < NUMBER_OF_ADC_CHANNEL; u8CountChannel++)
                    {
                        if (u16AdcValuesMax[u8CountChannel] < AverageAdcValues(u8CountChannel))
                        {
                            u16AdcValuesMax[u8CountChannel] = AverageAdcValues(u8CountChannel);
                        }
                        if (u16AdcValuesMin[u8CountChannel] > AverageAdcValues(u8CountChannel))
                        {
                            u16AdcValuesMin[u8CountChannel] = AverageAdcValues(u8CountChannel);
                        }
                    }
                }
                for (u8CountChannel = 0; u8CountChannel < NUMBER_OF_ADC_CHANNEL; u8CountChannel++)
                {
                    u16AdcValuesTemp[u8CountChannel] = (u16AdcValuesMax[u8CountChannel] + u16AdcValuesMin[u8CountChannel])/2U;
                }

                /* write new data to flash*/
                Flash_Write(FLASH_ADDRESS, (uint8_t *)u16AdcValuesTemp, NUMBER_OF_ADC_CHANNEL*2U);
                Flash_Read(FLASH_ADDRESS, (uint8_t *)u16AverageLines, NUMBER_OF_ADC_CHANNEL*2U);
                /* Not need to read to flash driver becasue it already read */
                ReadFlashOnce = FALSE;
                for (u8Count = 0; u8Count < NUMBER_OF_ADC_CHANNEL; u8Count++)
                {
                    if(u16AverageLines[u8Count] != u16AdcValuesTemp[u8Count])
                    {
                        return ROBOT_ERRORS;
                    }
                }

                WhileBackgroundTemp = CheckWhileBackground(&u16AverageLines[0]);
                Flash_Write(FLASH_ADDRESS + NUMBER_OF_ADC_CHANNEL*2U, (uint8_t *)&WhileBackgroundTemp, 2U);
                Flash_Read(FLASH_ADDRESS + NUMBER_OF_ADC_CHANNEL*2U, (uint8_t *)&InfoRobot.WhileBackground, 2U);

                if (InfoRobot.WhileBackground != WhileBackgroundTemp)
                {
                    return ROBOT_ERRORS;
                }
            }
        }
        else
        {
            /* Only need to read flash driver once when power up */
            if (ReadFlashOnce == TRUE)
            {
                Flash_Read(FLASH_ADDRESS, (uint8_t *)u16AverageLines, NUMBER_OF_ADC_CHANNEL*2U);
                Flash_Read(FLASH_ADDRESS + NUMBER_OF_ADC_CHANNEL*2U, (uint8_t *)&InfoRobot.WhileBackground, 2U);
                ReadFlashOnce = FALSE;
            }
        }
    }
    else
    {
        /* Only need to read flash driver once when power up */
        if (ReadFlashOnce == TRUE)
        {
            Flash_Read(FLASH_ADDRESS, (uint8_t *)u16AverageLines, NUMBER_OF_ADC_CHANNEL*2U);
            Flash_Read(FLASH_ADDRESS + NUMBER_OF_ADC_CHANNEL*2U, (uint8_t *)&InfoRobot.WhileBackground, 2U);
            ReadFlashOnce = FALSE;
        }
    }

    return RobotStatus;

}

Boolean CheckWhileBackground(uint16_t *pAdcValuesTemp)
{
    uint32_t u32Count = 0;
    uint8_t u8CountChannel = 0;
    uint32_t u32CountWhileOrBlack = 0;
    Boolean bWhileBackgroundRet = FALSE;

    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1U)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        Delay(500000);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        Delay(500000);
    }
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    /* Wait one time to prera */
    Delay(1000000);

    for (u32Count = 0U; u32Count < CHECK_BACKGROUND; u32Count++)
    {
        for (u8CountChannel = 0; u8CountChannel < NUMBER_OF_ADC_CHANNEL; u8CountChannel++)
        {
            if (pAdcValuesTemp[u8CountChannel] > AverageAdcValues(u8CountChannel))
            {
                u32CountWhileOrBlack++;
            }
        }
    }
    if (u32CountWhileOrBlack == (NUMBER_OF_ADC_CHANNEL*CHECK_BACKGROUND))
    {
        /* Turn off led of C13 pin */
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        bWhileBackgroundRet = FALSE;
    }
    else if (u32CountWhileOrBlack == 0U)
    {
        /* Turn off led of C13 pin */
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        bWhileBackgroundRet = TRUE;
    }
    else
    {
        /* Turn off led of C13 pin */
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        while(1);
    }

    return bWhileBackgroundRet;
}
void SysTick_Handler(void)
{
	 /* Get information of each lines */
   GetInfoOfEachLines(&u16AverageSensorLine[0], &InfoRobot);
   PD_Control(40, 40);
   SpeedMotor(u8LeftSpeed, 0, u8RightSpeed, 0);
}
void DMA_ConfigChannel_1(uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTranfer)
{
    /* Enable clock for DMA1 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /*
    * Set the peripheral register address in the DMA_CPARx register.
    * The data will be moved from/ to this address to/ from the memory after the peripheral event
    */
    DMA1_Channel1->CPAR = (uint32_t)pStartAddress;
    /*
    * Set the memory address in the DMA_CMARx register. The data will be written to or
    * read from this memory after the peripheral event
    */
    DMA1_Channel1->CMAR = (uint32_t)pDestination;
    /*
    *Configure the total number of data to be transferred in the DMA_CNDTRx register.
    * After each peripheral event, this value will be decremented
    */
    DMA1_Channel1->CNDTR = u32NumberDataTranfer;
    /*
     * Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
    */
    /*
    Configure data transfer direction, circular mode, peripheral & memory incremented
    mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
    DMA_CCRx register.
    */
    DMA1_Channel1->CCR |= 0x25A0;
    /* Activate the channel by setting the ENABLE bit in the DMA_CCRx register. */
    DMA1_Channel1->CCR |= 0x01;

}
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

    /* ==Configure ADC pins (PA0 -> Channel 0 to PA4 -> Channel 7) as analog inputs== */
    GPIO_InitStructure.GPIO_Pin = LINE_DETECTOR_SENSOR_OUT1 | LINE_DETECTOR_SENSOR_OUT2| LINE_DETECTOR_SENSOR_OUT3| LINE_DETECTOR_SENSOR_OUT4| LINE_DETECTOR_SENSOR_OUT5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		/* Buton to detect sensor lines */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8| GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void ADC_Config(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    /* -- Enable clock for ADC1 and GPIOA -- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* ADC1 configuration */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    /*We will convert multiple channels */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    /*select continuous conversion mode */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    /*select no external triggering */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    /*right 12-bit data alignment in ADC data register */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    /*8 channels conversion */
    ADC_InitStructure.ADC_NbrOfChannel = 5;
    /* load structure values to control and status registers */
    ADC_Init(ADC1, &ADC_InitStructure);
    /*configure each channel */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
    /*  Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* enable DMA for ADC */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/**********************************************************************************************************************
*                                              FLASH
***********************************************************************************************************************/
void Flash_Unlock(void)
{
    /* This sequence consists of two write cycles, where two key values (KEY1 and KEY2) are written to the FLASH_KEYR address*/
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
}

void Flash_Erase(volatile uint32_t u32StartAddr)
{
    /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
    while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY);

    /* Check unlock sequences */
    if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
    {
        Flash_Unlock();
    }

    /* Set the PER bit in the FLASH_CR register */
    FLASH->CR |= FLASH_CR_PER;
    /* Program the FLASH_AR register to select a page to erase */
    FLASH->AR = u32StartAddr;
    /* Set the STRT bit in the FLASH_CR register */
    FLASH->CR |= FLASH_CR_STRT;
    /* Wait for the BSY bit to be reset */
    while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY);
    /* Clear PER bit in the FLASH_CR register */
    FLASH->CR &= ~(uint32_t)FLASH_CR_PER;
    /* Clear STRT bit in the FLASH_CR register */
    FLASH->CR &= ~(uint32_t)FLASH_CR_STRT;

}
void SysTick_Interrupt(void)
{
	/* Enable Interrupt 1ms
   1 tick = 1/72.000.000 s
	  72000 -> 1/1000 s          
	*/
		SysTick->LOAD = 72U * 1000U - 1U;
		SysTick->VAL = 0U;
		SysTick->CTRL = 7U;
}
void Flash_Write(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length)
{
    uint32_t u32Count = 0U;

    /* Check input paras */
    if((u8BufferWrite == 0x00U) || (u32Length == 0U) || u32Length%2U != 0U)
    {
        return;
    }
    /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
    while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
    {
        /*  Wating for Bsy bit */
    }
    /* Check unlock sequences */
    if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
    {
        Flash_Unlock();
    }

    /* Write FLASH_CR_PG to 1 */
    FLASH->CR |= FLASH_CR_PG;

    /* Perform half-word write at the desired address*/
    for(u32Count = 0U; u32Count < (u32Length/2); u32Count ++ )
    {
        *(uint16_t*)(u32StartAddr + u32Count*2U) = *(uint16_t*)((uint32_t)u8BufferWrite + u32Count*2U);
        /* Wait for the BSY bit to be reset */
        while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY);
    }

    /* Clear PG bit in the FLASH_CR register */
    FLASH->CR &= ~(uint32_t)FLASH_CR_PG;

}
void Flash_Read(volatile uint32_t u32StartAddr, uint8_t* u8BufferRead, uint32_t u32Length)
{

    /* Check input paras */
    if((u8BufferRead == 0x00U) || (u32Length == 0U))
    {
        return;
    }
    do
    {
        if(( u32StartAddr%4U == 0U) && ((uint32_t)u8BufferRead%4U == 0U) && (u32Length >= 4U))
        {
            *(uint32_t*)((uint32_t)u8BufferRead) = *(uint32_t*)(u32StartAddr);
            u8BufferRead = u8BufferRead + 4U;
            u32StartAddr = u32StartAddr + 4U;
            u32Length = u32Length - 4U;
        }
        else if(( u32StartAddr%2U == 0U) && ((uint32_t)u8BufferRead%2U == 0U) && (u32Length >= 2U))
        {
            *(uint16_t*)((uint32_t)u8BufferRead) = *(uint16_t*)(u32StartAddr);
            u8BufferRead = u8BufferRead + 2U;
            u32StartAddr = u32StartAddr + 2U;
            u32Length = u32Length - 2U;
        }
        else
        {
            *(uint8_t*)(u8BufferRead) = *(uint8_t*)(u32StartAddr);
            u8BufferRead = u8BufferRead + 1U;
            u32StartAddr = u32StartAddr + 1U;
            u32Length = u32Length - 1U;
        }
    }
    while(u32Length > 0U);
}
void SpeedMotor(uint8_t SpeedMotor1, uint8_t SpeedMotor2, uint8_t SpeedMotor3, uint8_t SpeedMotor4)
{
	/* TIMx capture/compare register 1 (TIMx_CCR1) -> PB6 */
	TIM4->CCR1 = SpeedMotor1;
	/* TIMx capture/compare register 1 (TIMx_CCR2) -> PB7 */
	TIM4->CCR2 = SpeedMotor2;
	/* TIMx capture/compare register 1 (TIMx_CCR3) -> PB8 */
	TIM4->CCR3 = SpeedMotor3;
	/* TIMx capture/compare register 1 (TIMx_CCR4) -> PB9 */
	TIM4->CCR4 = SpeedMotor4;
}
void PWM_Timer4_Muti_Channels(void)
{
	/* Fre = 10Khz */
	TIM_TimeBaseInitTypeDef tim4Init;
	TIM_OCInitTypeDef pwmInit;
	
	/* timer 4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	tim4Init.TIM_Prescaler = 72 - 1;
	tim4Init.TIM_CounterMode = TIM_CounterMode_Up;
	tim4Init.TIM_Period = 100 - 1;
	tim4Init.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &tim4Init);
	TIM_Cmd(TIM4, ENABLE);
	
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	pwmInit.TIM_Pulse = 0;
	TIM_OC1Init(TIM4, &pwmInit);
	
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	pwmInit.TIM_Pulse = 0;
	TIM_OC2Init(TIM4, &pwmInit);
	
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	pwmInit.TIM_Pulse = 0;
	TIM_OC3Init(TIM4, &pwmInit);
	
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	pwmInit.TIM_Pulse = 0;
	TIM_OC4Init(TIM4, &pwmInit);
}
void Delay(uint32_t u32Timeout)
{
    while(u32Timeout > 0)
    {
        u32Timeout--;
    }
}
