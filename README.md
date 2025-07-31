# 💡 Blinking LED2 on Nucleo STM32F401RE

In this project i am going to make the Led2 on the nucleo stm32f401re board blink. ✨

I used Stm32Cube for the following steps.

First of all we go to the hardware configuration file "Pinout and configuration" to set PA5 as GPIO_Output (this pin is connected to LED2 that we want to blink).

Then we save the project to generate code. In the generated code we go in to while loop ("User code begin 3") and write the above code:
 
    HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 1); // PA5
    HAL_Delay(1000);
    HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 0); // PA5
    HAL_Delay(1000);

📕 void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

This function directly writes to the GPIO output data register (ODR or BSRR) to change the logic level of the pin.

💠 GPIOx : The GPIO port where the pin is located, e.g. GPIOA, GPIOB, etc.

💠 GPIO_Pin : The specific pin number you want to write to, e.g. GPIO_Pin_5.

💠 Pinstate : The value to write.

📕 void HAL_Delay();

💠 HAL_Delay uses the SysTick timer (typically configured in HAL_Init() ) to create a delay.

   --> The SysTick timer is a core peripheral in ARM Cortex-M microcontrollers that provides a simple way to create periodic interrupts or delays. 
   
💠 It is a blocking function, meaning the CPU will stop executing other tasks until the delay is complete.

 Some notes here that are important
 
 :one: We have to write the user code within the commented area because when we build the project only this part of code is going to be saved.
 
 2️⃣ We use HAL(Hardware Abstraction Level) library
 
 3️⃣ We could do it without the use of HAL library in a more complex, bare metal way like this:
 
    // 1. Enable GPIOA clock (bit 0 of AHB1ENR)
    RCC->AHB1ENR |= (1<<0); // or RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // 2. Set PA5 as output (MODER5 = 0b01)
    GPIOA->MODER &= ~(3 << (5 * 2)); // Clear mode bits for PA5
    GPIOA->MODER |=  (1 << (5 * 2)); // Set as general purpose output
    while (1)
    {
        // 3. Set PA5 HIGH
        GPIOA->ODR |= (1 << 5);
        delay_ms(1000);
        // 4. Set PA5 LOW
        GPIOA->ODR &= ~(1 << 5);
        delay_ms(1000);
    }
    }
    // Simple software delay (approximate)
    void delay_ms(uint32_t ms)
    {
      for(uint32_t i = 0; i < ms * 4000; i++) __NOP();
    }
After we build the code and run it, if the board is connected to your PC.

Now we should see the LED blinking. 🔦
