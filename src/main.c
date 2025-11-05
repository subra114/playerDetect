/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 15, 2024
  * @brief   ECE 362 Lab 2 Student template
  ******************************************************************************
*/


/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "subra114";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>

void initc();
// void initb();
// void togglexn(GPIO_TypeDef *port, int n);
void init_exti();
void set_col(int col);
void SysTick_Handler();
void init_systick();

extern void internal_clock();
extern void nano_wait(int);
int numPlayers = 0; //number of players enabled

int main(void) {
    internal_clock();
    // initb();
    initc();
    init_exti();
    init_systick();
    adjust_priorities();

    // Slowly blinking
    for(;;) {
        // togglexn(GPIOC, 9);
        nano_wait(500000000);
    }
}

/**
 * @brief Init GPIO port C
 *        PC0-PC3 as input pins with the pull up resistor enabled
 *        PC6-PC9 as output pins
 * 
 */
void initc() {
  //init port
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  //set input (even though its default just incase)
  GPIOC->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);


  //set output
  
  GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
  GPIOC->MODER &= ~(GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);

  //set pull up

  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_0;

  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_0;

  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR2_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR2_0;
  
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3_1;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR3_0;
}

/**
 * @brief Init GPIO port B
 *        PB0, PB2, PB3, PB4 as input pins
 *          enable pull down resistor on PB2 and PB3
 *        PB8-PB11 as output pins
 * 
 */
// void initb() {
//   RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//   GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2_0;
//   GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1;
  
//   GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR3_0;
//   GPIOB->PUPDR |= GPIO_PUPDR_PUPDR3_1;

//   GPIOB->MODER &= ~GPIO_MODER_MODER0_0;
//   GPIOB->MODER &= ~GPIO_MODER_MODER0_1;

//   GPIOB->MODER &= ~GPIO_MODER_MODER4_0;
//   GPIOB->MODER &= ~GPIO_MODER_MODER4_1;

//   GPIOB->MODER |= GPIO_MODER_MODER8_0; // |= turns on
//   GPIOB->MODER &= ~GPIO_MODER_MODER8_1; // &= ~ turns off

//   GPIOB->MODER |= GPIO_MODER_MODER9_0;
//   GPIOB->MODER &= ~GPIO_MODER_MODER9_1;

//   GPIOB->MODER |= GPIO_MODER_MODER10_0;
//   GPIOB->MODER &= ~GPIO_MODER_MODER10_1;

//   GPIOB->MODER |= GPIO_MODER_MODER11_0;
//   GPIOB->MODER &= ~GPIO_MODER_MODER11_1;
// }

/**
 * @brief Change the ODR value from 0 to 1 or 1 to 0 for a specified 
 *        pin of a port.
 * 
 * @param port : The passed in GPIO Port
 * @param n    : The pin number
 */
// void togglexn(GPIO_TypeDef *port, int n) {
//   int32_t temp = 1<<n;
//   int32_t origVal = port->IDR & temp;

//   if (origVal== 0) {
//     port->BSRR |= (1<<n);
//   }
//   else {
//     port->BSRR |= (1<<(n+16));
//   }
// }

/**
 * @brief Follow the lab manual to initialize EXTI.  In a gist:
 *        (1-2) Enable the SYSCFG subsystem, and select Port C for
 *            pins 0, 1, 2, and 3.
 *        (3) Configure the EXTI_RTSR and EXTI_FTSR register so that an EXTI
 *            interrupt is generated on the rising edge  and falling edge of 
 *            each of the pins.
 *        (4) Configure the EXTI_IMR register so that the EXTI
 *            interrupts are unmasked for each of the pins.
 *        (5) Enable the three interupts for EXTI pins 0-1, and 2-3.
 *            Don't enable any other interrupts.
 */
void init_exti() { 

  // turn clock on for external interrupts
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
  
  //turn on pins 0-3 for interupts
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PC;
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PC;
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PC;
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PC; 
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC; 

  //configure for rising edge
  EXTI->RTSR |= EXTI_RTSR_TR0;
  EXTI->RTSR |= EXTI_RTSR_TR1;
  EXTI->RTSR |= EXTI_RTSR_TR2;
  EXTI->RTSR |= EXTI_RTSR_TR3;

  //configure for falling edge
  EXTI->FTSR |= EXTI_FTSR_FT0;
  EXTI->FTSR |= EXTI_FTSR_FT1;
  EXTI->FTSR |= EXTI_FTSR_FT2;
  EXTI->FTSR |= EXTI_FTSR_FT3;

  //unmasking pins
  EXTI->IMR |= EXTI_IMR_IM0;  
  EXTI->IMR |= EXTI_IMR_IM1;
  EXTI->IMR |= EXTI_IMR_IM2;
  EXTI->IMR |= EXTI_IMR_IM3;

  //enable interrupts
  NVIC->ISER[0] |= 1<<EXTI0_1_IRQn;
  NVIC->ISER[0] |= 1<<EXTI2_3_IRQn;

}

//==========================================================
// Write the EXTI interrupt handler for pins 0 and 1 below.
// Acknowledge the pending bit for pin 0, check which of the pins
// changed their values, update number of players, change the corresponding led
// accordingly, print number of players
void EXTI0_1_IRQHandler () {
  //acknowledge pending bit
  EXTI->PR |= EXTI_PR_PR0;

  //pin 0 != pin 6
  if ((GPIOC->IDR & (1<<0)) != (GPIOC->IDR & (1<<6))){
    if (GPIOC->IDR & (1<<0)) { //pin 0 is high
      numPlayers++; //increment number of players
      GPIOC->BSRR |= (1<<(6+16)); //set led 6 high
    }
    else { //pin 0 is low
      numPlayers--; //decrement number of players
      GPIOC->BSRR |= (1<<(6)); //set led 6 low
    }
  }
  else { //pin 1 != pin 7
    if (GPIOC->IDR & (1<<1)) { //pin 1 is high
      numPlayers++; //increment number of players
      GPIOC->BSRR |= (1<<(7+16)); //set led 7 high
    }
    else { //pin 1 is low
      numPlayers--; //decrement number of players
      GPIOC->BSRR |= (1<<(7)); //set led 7 low
    }
  }

  //print number of players
  printf("Number of players: %d\n", numPlayers);

}


//==========================================================
// Write the EXTI interrupt handler for pins 2 and 3 below.
// Acknowledge the pending bit for pin 2, check which of the pins
// changed their values, update number of players, change the corresponding led
// accordingly, print number of players
void EXTI2_3_IRQHandler () {
  //aknowledge bit
  EXTI->PR |= EXTI_PR_PR2;

  //pin 2 != pin 8
  if ((GPIOC->IDR & (1<<2)) != (GPIOC->IDR & (1<<8))){
    if (GPIOC->IDR & (1<<2)) { //pin 2 is high
      numPlayers++; //increment number of players
      GPIOC->BSRR |= (1<<(2+16)); //set led 8 high
    }
    else { //pin 2 is low
      numPlayers--; //decrement number of players
      GPIOC->BSRR |= (1<<(2)); //set led 8 low
    }
  }
  else { //pin 3 != pin 9
    if (GPIOC->IDR & (1<<3)) { //pin 3 is high
      numPlayers++; //increment number of players
      GPIOC->BSRR |= (1<<(9+16)); //set led 9 high
    }
    else { //pin 3 is low
      numPlayers--; //decrement number of players
      GPIOC->BSRR |= (1<<(9)); //set led 9 low
    }
  }

  //print number of players
  printf("Number of players: %d\n", numPlayers);
}
//==========================================================



// /**
//  * @brief Enable the SysTick interrupt to occur every 1/16 seconds.
//  * 
//  */
// void init_systick() { //TA HELP HERE PLEASE
//   SysTick->LOAD = 374999;
//   SysTick->VAL = 0;
//   SysTick->CTRL |= ( SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
//   SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
// }

// volatile int current_col = 1;

// /**
//  * @brief The ISR for the SysTick interrupt.
//  * 
//  */
// void SysTick_Handler() {
//   for (int row = 3; row >= 0; row--) {
//     GPIOC->BSRR |= 1<<(8 - current_col);
//     int32_t tempRead = 1<<row;

//     if (GPIOC->IDR & tempRead == 0) {
//       GPIOB->BSRR |= 1<<((7 + current_col)+16);

//     }
//     else {
//       GPIOB->BSRR |= 1<<(7 + current_col);
//     }
    
//     if (current_col == 4) current_col = 0;
//     if (row < 0) row = 4;
//     current_col++;
//   }
// }
