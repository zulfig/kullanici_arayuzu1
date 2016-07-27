
/* figur_Kutuphane_REG*/
/* Register i�lemleri i�in tan�mlar*/
#define SET_BIT(REG, BIT)    ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)  ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)   ((REG) & (BIT))
#define CLEAR_REG(REG)       ((REG) = (0x0))
#define WRITE_REG(REG, VAL)  ((REG) = (VAL))
#define READ_REG(REG)        ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)   WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
/* �rne�in: CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk); */
/*  RCC->CR |= (uint32_t)0x00000001U;*/
/*  SET_BIT (RCC-CR, (uint32_t)0x00000001U);  YER�NE KONAN BU*/
/*  WRITE_REG (REG, VAL) tan�m�n� kullanarak  ARR de�eri atamas� */
/*  WRITE_REG (TIM1_ARR, 1200)*/