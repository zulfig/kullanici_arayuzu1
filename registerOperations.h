
/* figur_Kutuphane_REG*/
/* Register iþlemleri için tanýmlar*/
#define SET_BIT(REG, BIT)    ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)  ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)   ((REG) & (BIT))
#define CLEAR_REG(REG)       ((REG) = (0x0))
#define WRITE_REG(REG, VAL)  ((REG) = (VAL))
#define READ_REG(REG)        ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)   WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
/* Örneðin: CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk); */
/*  RCC->CR |= (uint32_t)0x00000001U;*/
/*  SET_BIT (RCC-CR, (uint32_t)0x00000001U);  YERÝNE KONAN BU*/
/*  WRITE_REG (REG, VAL) tanýmýný kullanarak  ARR deðeri atamasý */
/*  WRITE_REG (TIM1_ARR, 1200)*/