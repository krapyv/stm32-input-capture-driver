#include "stm32f411.h"
#include "core_cm4.h"
#include "input_capture.h"

IC_HandleTypeDef input_capture; // memory allocated

void TIM3_FirstEdgeHandler(uint32_t polarity_lsb, uint32_t polarity_msb)
{
    input_capture.data.first_timestamp = input_capture.Instance->CCR[input_capture.channel - 1];

    // Toggle the polarity bits to the opposite of whatever they currently are
    uint32_t current_polarity = (input_capture.Instance->CCER >> polarity_lsb) & 0x1;

    input_capture.Instance->CCER &= ~((1 << polarity_msb) | (1 << polarity_lsb));

    // if current_polarity is 0 (rising), make it 1 (falling)
    // if it was 1 (falling), switch it to 0 (rising)
    if (current_polarity == 0)
    {
        // falling edge is 01, so we need to set the LSB to 1
        input_capture.Instance->CCER |= (1 << polarity_lsb);
    }
}

void TIM3_SecondEdgeHandler(uint32_t polarity_lsb, uint32_t polarity_msb)
{
    input_capture.data.second_timestamp = input_capture.Instance->CCR[input_capture.channel - 1];

    uint32_t total_overflow_ticks = 0;
    if (input_capture.Instance == TIM2 || input_capture.Instance == TIM5)
    {
        // the overflow for these times takes almost 4.5 minutes
        // so we do not actually need to track it in this driver
        total_overflow_ticks = input_capture.data.overflow_count * 0;
    }
    else
    {
        // TIM3 and TIM4 are 16-bit timers, so the max value they hold is 2^16
        total_overflow_ticks = (input_capture.data.overflow_count << 16); // 2^16 = 65536
    }

    input_capture.data.delta = total_overflow_ticks + (input_capture.data.second_timestamp - input_capture.data.first_timestamp);

    // reset tracking data blocks
    input_capture.data.overflow_count = 0;
    input_capture.data.first_timestamp = 0;
    input_capture.data.second_timestamp = 0;

    // the total_overflow_ticks is in scope of this else block, so no need to set in to 0

    // restore the original configuration
    // clearing the bits
    input_capture.Instance->CCER &= ~((1 << polarity_msb) | (1 << polarity_lsb));

    // if it was IC_RE (rising edge), clearing the bits restored its value of 00
    if (input_capture.polarity == IC_FE)
    {
        input_capture.Instance->CCER |= (1 << polarity_lsb);
    }
    else if (input_capture.polarity == IC_BE)
    {
        // falling edge is 01, so we need to set the LSB to 1
        input_capture.Instance->CCER |= ((1 << polarity_msb) | (1 << polarity_lsb));
    }
}
void TIM3_IRQHandler(void)
{
    uint32_t ccif_mask = (1 << input_capture.channel);

    uint32_t sr_var = input_capture.Instance->SR;

    uint32_t polarity_lsb_pos = ((input_capture.channel - 1) * 4 + 1); // least significant bit
    uint32_t polarity_msb_pos = polarity_lsb_pos + 2;

    // overflow/update flags handling
    if (sr_var & (1 << 0))
    {
        input_capture.Instance->SR = ~(1 << 0);

        if (input_capture.data.first_timestamp != 0)
        {
            // count the overflow if measurements are in progress
            input_capture.data.overflow_count++;
        }
    }

    if (input_capture.Instance->SR & ccif_mask)
    {
        // additional protection
        // wiping out the status interrupt
        // update flag has been cleared right after it was processed

        // to prevent the situations when the new edge happens while we are processing the previous one, we are clearing the register immediately
        input_capture.Instance->SR = ~ccif_mask;

        if (input_capture.data.first_timestamp == 0)
        {
            TIM3_FirstEdgeHandler(polarity_lsb_pos, polarity_msb_pos);
        }
        else
        {
            TIM3_SecondEdgeHandler(polarity_lsb_pos, polarity_msb_pos);
        }
    }
}

void nvic_interrupts_init(IC_HandleTypeDef *ic)
{
    if (ic->Instance == TIM2)
    {
        NVIC->ISER[0] |= (1 << 28);
    }
    else if (ic->Instance == TIM3)
    {
        NVIC->ISER[0] |= (1 << 29);
    }
    if (ic->Instance == TIM4)
    {
        NVIC->ISER[0] |= (1 << 30);
    }
    if (ic->Instance == TIM5)
    {
        NVIC->ISER[1] |= (1 << 18); // 50 - 32 = 18
    }
}

void input_capture_init(IC_HandleTypeDef *ic)
{
    if (ic->pin > 15 || ic->channel > 4)
        return;

    // enabling the clock for the Port
    if (ic->Port == GPIOA)
    {
        RCC->AHB1ENR |= (1 << 0);
    }
    else if (ic->Port == GPIOB)
    {
        RCC->AHB1ENR |= (1 << 1);
    }

    // setting the MODER to AF (10)

    // every pin takes two bits in the MODER
    // e.g. pin 0 takes bits 0 and 1
    // so the least significant bit has a position: ic->pin * 2
    // e.g. pin 0 -> lsb: 0, pin 9 -> lsb: 9 * 2 = 18;

    // clearing the bits
    // 11 = 2^1 + 2^0 = 3 = 0x3
    ic->Port->MODER &= ~(0x3 << (ic->pin * 2));

    // setting the bits to 10
    // 10 = 2^1 = 0x2
    ic->Port->MODER |= (0x2 << (ic->pin * 2));

    // setting the PB0 to the passed AF value
    // each pin in both AFRL and AFRH takes 4 bits
    // e.g., pin 1 takes bits 7:4
    if (ic->pin < 8)
    {
        // clearing the bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        ic->Port->AFRL &= ~(0xF << (ic->pin * 4));

        // setting the bits to the passed AF value
        ic->Port->AFRL |= (ic->af << (ic->pin * 4));
    }
    else
    {
        // we are subtracking 8 from the pin number
        // to prevent overfloating of uint32_t (because e.g. pin number 8 * 4 = 32 => 0xF << 32 <----- does not make much sense)

        // clearing the bits
        // 1111 = 2^3 + 2^2 + 2^1 + 2^0 = 8 + 4 + 2 + 1 = 15 = 0xF
        ic->Port->AFRH &= ~(0xF << ((ic->pin - 8) * 4));

        // setting the bits to the passed AF value
        ic->Port->AFRH |= (ic->af << ((ic->pin - 8) * 4));
    }

    // enabling the clock for passed Instance
    if (ic->Instance == TIM2)
    {
        RCC->APB1ENR |= (1 << 0);
    }
    else if (ic->Instance == TIM3)
    {
        RCC->APB1ENR |= (1 << 1);
    }
    else if (ic->Instance == TIM4)
    {
        RCC->APB1ENR |= (1 << 2);
    }
    else if (ic->Instance == TIM5)
    {
        RCC->APB1ENR |= (1 << 3);
    }

    // setting bits in CCMR
    if (ic->channel == 1 || ic->channel == 2)
    {
        // setting CCxS bits to 01 (input, mapped on TIx)

        // if it is a channel 1 -> (1 - 1) * 8 = 0
        // if it is a channel 2 -> (2 - 1) * 8 = 8
        uint32_t ccs_pos = ((ic->channel - 1) * 8);

        // clearing the bits
        // 0x3 = 0011

        ic->Instance->CCMR1 &= ~(0x3 << ccs_pos);

        // setting the bits
        // 01 = 0x1
        ic->Instance->CCMR1 |= (0x1 << ccs_pos);

        // setting ICxF (filtering) to 1111 (fDTS / 32, N=8)
        // channel 1 - bits 7:4
        // channel 2 - bits 15:12

        // if channel 1 -> ((1 - 1) * 8)+ 4 = 4
        // if channel 2 -> ((2 - 1) * 8)+ 4 = 12
        uint32_t icf_pos = ((ic->channel - 1) * 8) + 4;

        // clearing the bits
        // 1111 = 15 = 0xF
        ic->Instance->CCMR1 &= ~(0xF << icf_pos);

        // setting the bits
        // 1111 = 0xF
        ic->Instance->CCMR1 |= (0xF << icf_pos);
    }
    else
    {
        // setting CCxS bits to 01 (input, mapped on TIx)

        // if it is a channel 3 -> (3 - 3) * 8 = 0
        // if it is a channel 4 -> (4 - 3) * 8 = 8
        uint32_t ccs_pos = ((ic->channel - 3) * 8);

        // clearing the bits
        // 0x3 = 0011
        ic->Instance->CCMR2 &= ~(0x3 << ccs_pos);

        // setting the bits
        // 01 = 0x1
        ic->Instance->CCMR2 |= (0x1 << ccs_pos);

        // setting ICxF (filtering) to 1111 (fDTS / 32, N=8)
        // channel 3 - bits 7:4
        // channel 4 - bits 15:12

        // if channel 3 -> ((3 - 3) * 8) + 4 = 4
        // if channel 4 -> ((4 - 3) * 8) + 4 = 8 + 4 = 12
        uint32_t icf_pos = ((ic->channel - 3) * 8) + 4;

        // clearing the bits
        // 1111 = 15 = 0xF
        ic->Instance->CCMR2 &= ~(0xF << icf_pos);

        // setting the bits
        // 1111 = 0xF
        ic->Instance->CCMR2 |= (0xF << icf_pos);
    }

    // set bits in CCER

    // enable capture/compare output for corresponding channel (set to 1)
    // channel 1 - bit 0; channel 2 - bit 4; channel 3 - bit 8; channel 4 - bit 12;
    uint32_t cce_pos = (ic->channel - 1) * 4;

    // setting the bit
    ic->Instance->CCER |= (1 << cce_pos);

    // set the polarity
    // channel 1 - bits 1 and 3; channel 2 - bits 5 and 7; channel 3 - bits 9 and 11; channel 4 - bits 13 and 15;

    // channel 1 - (1 - 1) * 4 + 1 = 1
    // channel 2 - (2 - 1) * 4 + 1 = 5
    // channel 3 - (3 - 1) * 4 + 1 = 9
    // channel 4 - (4 - 1) * 4 + 1 = 13
    uint32_t polarity_lsb_pos = ((ic->channel - 1) * 4 + 1); // least significant bit
    uint32_t polarity_msb_pos = polarity_lsb_pos + 2;        // most significant bit

    // setting the polarity

    // clearing the bits
    ic->Instance->CCER &= ~((1 << polarity_msb_pos) | (1 << polarity_lsb_pos));

    // since the rising edge is 00, we do not need to set any other bit after clearing

    if (ic->polarity == IC_FE)
    {
        // falling edge is 01, so we need to set the LSB to 1
        ic->Instance->CCER |= (1 << polarity_lsb_pos);
    }
    else if (ic->polarity == IC_BE)
    {
        // both edges is 11, so we need to set both bits to 1
        ic->Instance->CCER |= ((1 << polarity_msb_pos) | (1 << polarity_lsb_pos));
    }

    // enabling Capture/Compare Interrupt and Update Interrupt (to keep track of very long button presses)
    ic->Instance->DIER |= (1 << ic->channel) | (1 << 0);

    // enable the counter in CR1 (setting bit 0 CEN to 1)
    ic->Instance->CR1 |= (1 << 0);
}
