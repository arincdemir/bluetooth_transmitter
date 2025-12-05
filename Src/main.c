#include <stdint.h>

// Struct Definitions

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
    volatile uint32_t BRR;
    uint32_t reserved;
    volatile uint32_t SECCFGR;
} GPIO_Struct;

typedef struct {
    volatile uint32_t CR1;   // 0
    volatile uint32_t CR2;   // 4
    volatile uint32_t CR3;   // 8
    volatile uint32_t BRR;   // C
    uint32_t reserved1[2];   // 10 14
    volatile uint32_t RQR;   // 18
    volatile uint32_t ISR;   // 1C
    volatile uint32_t ICR;   // 20
    volatile uint32_t RDR;   // 24
    volatile uint32_t TDR;   // 28
    volatile uint32_t PRESC; // 2C
} LPUARTType;

typedef struct {
    volatile uint32_t CR1;   // 0
    volatile uint32_t CR2;   // 4
    volatile uint32_t CR3;   // 8
    volatile uint32_t BRR;   // C
    volatile uint32_t GTPR;  // 10
    volatile uint32_t RTOR;  // 14
    volatile uint32_t RQR;   // 18
    volatile uint32_t ISR;   // 1C
    volatile uint32_t ICR;   // 20
    volatile uint32_t RDR;   // 24
    volatile uint32_t TDR;   // 28
    volatile uint32_t PRESC; // 2C
} USARTType;

// Memory address macros

#define LPUART1 ((LPUARTType *) 0x40008000)
#define USART3 ((USARTType *) 0x40004800)


#define GPIOB_PTR       ((GPIO_Struct *) 0x42020400)
#define GPIOG_PTR 	((GPIO_Struct *) 0x42021800)

#define ISER1 *((volatile uint32_t*) 0xE000E104)

#define PWR_CR1 *((volatile uint32_t *) 0x40007000)
#define PWR_CR2 *((volatile uint32_t *) 0x40007004)

#define RCC_CFGR *((volatile uint32_t *) 0x40021008)
#define RCC_CR *((volatile uint32_t *) 0x40021000)
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define RCC_APB1ENR2 *((volatile uint32_t *) 0x4002105C)
#define RCC_APB2ENR *((volatile uint32_t *) 0x40021060)
#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)

#define NVIC_ISER1 *((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2 *((volatile uint32_t*) 0xE000E108)


// Initialize LPUART1 for communicating serially with the computer using USB.
void LPUART1_initialization(void) {
    // Enable Clock
    RCC_APB1ENR1 |= (1 << 28);

    // Change the regulator mode to Low-power mode.
    PWR_CR1 |= (1 << 14);

    // Turn VDDIO2 for PG[15:2] pins.
    PWR_CR2 |= (1 << 9);

    // Select SYSCLK (=4MHz) for the clock source of Low-power UART.
    RCC_CCIPR1 &= ~(1 << 11);
    RCC_CCIPR1 |= (1 << 10);

    // Enable GPIO G port for Tx and Rx pins.
    RCC_AHB2ENR |= (1 << 6);

    // Set alternate function for PG7 and PG8.
    GPIOG_PTR->MODER &= ~(0b0101 << (7*2));
    GPIOG_PTR->MODER |= (0b1010 << (7*2));

    // Connect PG7 to LPUART Tx.
    GPIOG_PTR->AFRL &= ~(0b0111 << (7*4));
    GPIOG_PTR->AFRL |= (0b1000 << (7*4));

    // Connect PG8 to LPUART Rx. (Original text said Tx, but PG8 is usually RX in this context)
    GPIOG_PTR->AFRH &= ~0b0111;
    GPIOG_PTR->AFRH |= 0b1000;

    // Enable Clock for LPUART.
    RCC_APB1ENR2 |= 1;

    // Set LPUART baud rate (BRR) for 115200 baud rate.
    // 256 * 4000000 / 115200 for 115200 baud rate.
    LPUART1->BRR = 8888;

    // Enable FIFO.
    LPUART1->CR1 |= (1 << 29);

    // Enable Transmitter and Receiver.
    LPUART1->CR1 |= (0b11 << 2);

    // Enable interrupt for FIFO is not empty.
    LPUART1->CR1 |= (1 << 5);

    // Enable global signalling from ISER.
    NVIC_ISER2 |= (1 << 2); // ISER2 bit 2 corresponds to LPUART1 global interrupt (IRQ 66)

    // Enable LPUART.
    LPUART1->CR1 |= 1;
}


// Initialize USART3 for communicating with the HC-05 Bluetooth module.
void USART3_initialization(void) {
		// Enable Clocks
	     RCC_AHB2ENR |= (1 << 1);               // Enable GPIOB Clock (Bit 1)
	     RCC_APB1ENR1 |= 1 << 18; // Enable USART3 Clock

	     // Configure PB10 (TX) and PB11 (RX)
	     // Clear Mode Bits for Pin 10 and 11
	     GPIOB_PTR->MODER &= ~((3 << (10*2)) | (3 << (11*2)));
	     // Set to Alternate Function Mode (10)
	     GPIOB_PTR->MODER |=  ((2 << (10*2)) | (2 << (11*2)));

	     // Set Alternate Function to AF7 (USART3)
	     // PB10 is in AFRH (pins 8-15). PB10 is bits [11:8], PB11 is bits [15:12]
	     GPIOB_PTR->AFRH &= ~((0xF << 8) | (0xF << 12)); // Clear old settings
	     GPIOB_PTR->AFRH |=  ((0x7 << 8) | (0x7 << 12)); // Set AF7 (0111)

	     // Set Baud Rate to 38400 (HC-05 Default)
	     // Formula: F_clk / Baud. F_clk is 4MHz
	     // 4,000,000 / 38400 ~= 104.16. Round to 104 (0x68).
	     USART3->BRR = 104;

	     // Enable RX Interrupt (RXNEIE)
	     USART3->CR1 |= (1 << 5);

	     // Enable UART (TE=1, RE=1, UE=1)
	     USART3->CR1 |= (1 << 3) | (1 << 2) | (1 << 0);

	     // Enable USART3 Interrupt in NVIC
	     NVIC_ISER1 |= (1 << 31);
}


// Read the data from the computer and forward it to HC-05
void LPUART1_IRQHandler(void) {
    // Check if data is ready to be read
    if ((LPUART1->ISR & (1 << 5)) != 0) {
        char data = (char)LPUART1->RDR;

        // Wait until USART2 Transmit Data Register is Empty (TXE flag, bit 7)
        while (!((USART3->ISR >> 7) & 1));

        // Send data to HC-05 via USART3
        USART3->TDR = data;

    }
}

// Read the data coming from the Bluetooth module and forward it to the computer.
void USART3_IRQHandler(void) {
    // Check if data is ready to be read (RXNE flag, bit 5)
    if ((USART3->ISR & (1 << 5)) != 0) {
        char data = (char)USART3->RDR;

        // Wait until LPUART1 Transmit Data Register is Empty (TXE flag, bit 7)
        while (!((LPUART1->ISR >> 7) & 1));

        // Send data to Computer via LPUART1
        LPUART1->TDR = data;

    }
}

void _enable_irq(void) {
    asm volatile(
        "mov r0, #0 \n\t"
        "msr primask, r0 \n\t"
    );
}

int main(void) {
    LPUART1_initialization();
    USART3_initialization();
    _enable_irq();

    while(1) {
        asm volatile("wfi");
    }
}
