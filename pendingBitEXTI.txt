/* figur_Kutuphane_GPIOA*/

/* GPIOA  Register adreslerini atayalým*/
#define GPIOA_MODER         (*((uint32_t*)0x48000000))
#define GPIOA_OTYPER        (*((uint32_t*)0x48000004))
#define GPIOA_OSPEEDR     (*((uint32_t*)0x48000008))
#define GPIOA_PUPDR          (*((uint32_t*)0x4800000C))
#define GPIOA_IDR               ( *((uint32_t*)0x48000010))
#define GPIOA_ODR             (* ((uint32_t*)0x48000014))
#define GPIOA_BSRR           (*( (uint32_t*)0x48000018))
#define GPIOA_LCKR            (*((uint32_t*)0x4800001C))
#define GPIOA_AFRL             (*((uint32_t*)0x48000020))
#define GPIOA_AFRH            (*((uint32_t*)0x48000024))
#define GPIOA_BRR              (*((uint32_t*)0x48000028))





