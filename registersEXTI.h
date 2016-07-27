/* figur_Kutuphane_EXTI*/
/* EXTI  Register adreslerini atayalým*/
#define EXTI_IMR      (*((uint32_t*)0x40010400))
#define EXTI_EMR     (*((uint32_t*)0x40010404))
#define EXTI_RTSR    (*((uint32_t*)0x40010408))
#define EXTI_FTSR    (*((uint32_t*)0x4001040C))
#define EXTI_SWIER  (*((uint32_t*)0x40010410))
#define EXTI_PR        (*((uint32_t*)0x40010414))


/* figur_Kutuphane_EXTI_PendingBits*/

#define  PendingBit0         (0x00000001) /* PA0-PB0-PC0-PD0 portlarý*/
#define  PendingBit1         (0x00000002)
#define  PendingBit2         (0x00000004)
#define  PendingBit3         (0x00000008)
#define  PendingBit4         (0x00000010)
#define  PendingBit5         (0x00000020)
#define  PendingBit6         (0x00000040)
#define  PendingBit7         (0x00000080)
#define  PendingBit8         (0x00000100)
#define  PendingBit9         (0x00000200)
#define  PendingBit10        (0x00000400)
#define  PendingBit11        (0x00000800)
#define  PendingBit12        (0x00001000)
#define  PendingBit13        (0x00002000)
#define  PendingBit14        (0x00004000)
#define  PendingBit15        (0x00008000)

