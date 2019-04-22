#include <stdint.h>




#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <string>

uint16_t read_adc_naiive(uint8_t channel);
#define USART_CONSOLE USART1


extern "C"{
/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
  int i;


    for (i = 0; i < len; i++) {

      usart_send_blocking(USART_CONSOLE, ptr[i]);
    }
    return i;
}
}


enum class STATE : uint8_t  { INIT, GO_UP, GO_DOWN  };

STATE state{STATE::INIT};

void init()
{
    uint16_t adcValue = read_adc_naiive(0);

    if(adcValue < 150)
    {
        state = STATE::GO_UP;
    }
}

void goUp()
{


}

void goDown()
{

}


/* milliseconds since boot */
volatile uint32_t system_millis;

/* Called when systick fires */
void sys_tick_handler(void)
{
  system_millis++;
}

/* simple sleep for delay milliseconds */
void msleep(uint32_t delay)
{
  uint32_t wake = system_millis + delay;
  while (wake > system_millis);
}

/* Getter function for the current time */
uint32_t mtime(void)
{
  return system_millis;
}

void tim_setup(void)
{
	/* Enable TIM1 clock. */
	rcc_periph_clock_enable(RCC_TIM1);

	/* Enable GPIOA, GPIOB and Alternate Function clocks. */
	rcc_periph_clock_enable(RCC_AFIO);

	/*
	 * Set TIM1 channel output pins to
	 * 'output alternate function push-pull'.
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 		      GPIO_TIM1_CH1 | GPIO_TIM1_CH2 | GPIO_TIM1_CH3 );
        
        
        /* Enable TIM1 commutation interrupt. */
	nvic_enable_irq(NVIC_TIM1_TRG_COM_IRQ);


	

	/* Reset TIM1 peripheral. */
	rcc_periph_reset_pulse(RST_TIM1);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/* Reset prescaler value. */
	timer_set_prescaler(TIM1, 0);

	/* Reset repetition counter value. */
	timer_set_repetition_counter(TIM1, 0);

	/* Enable preload. */
	timer_enable_preload(TIM1);

	/* Continuous mode. */
	timer_continuous_mode(TIM1);

	/* Period (20Hz). */
	timer_set_period(TIM1, 24000000 / 20);

	/* Configure break and deadtime. */
	timer_set_deadtime(TIM1, 10);
	timer_set_enabled_off_state_in_idle_mode(TIM1);
	timer_set_enabled_off_state_in_run_mode(TIM1);
	timer_disable_break(TIM1);
	timer_set_break_polarity_high(TIM1);
	timer_disable_break_automatic_output(TIM1);
	timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);

	

	/* ARR reload enable. */
	timer_enable_preload(TIM1);

	/*
	 * Enable preload of complementary channel configurations and
	 * update on COM event.
	 */
	timer_enable_preload_complementry_enable_bits(TIM1);

	/* Enable outputs in the break subsystem. */
	timer_enable_break_main_output(TIM1);

	/* Counter enable. */
	timer_enable_counter(TIM1);
        
        /* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC1);
	timer_disable_oc_output(TIM1, TIM_OC1N);

	/* Configure global mode of line 1. */
	timer_disable_oc_clear(TIM1, TIM_OC1);
	timer_enable_oc_preload(TIM1, TIM_OC1);
	timer_set_oc_slow_mode(TIM1, TIM_OC1);
	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);

	/* Configure OC1. */
	timer_set_oc_polarity_high(TIM1, TIM_OC1);
	timer_set_oc_idle_state_set(TIM1, TIM_OC1);

	/* Configure OC1N. */
	timer_set_oc_polarity_high(TIM1, TIM_OC1N);
	timer_set_oc_idle_state_set(TIM1, TIM_OC1N);

	/* Set the capture compare value for OC1. */
	timer_set_oc_value(TIM1, TIM_OC1, 100);

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_oc_output(TIM1, TIM_OC1N);

	/* -- OC2 and OC2N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC2);
	timer_disable_oc_output(TIM1, TIM_OC2N);

	/* Configure global mode of line 2. */
	timer_disable_oc_clear(TIM1, TIM_OC2);
	timer_enable_oc_preload(TIM1, TIM_OC2);
	timer_set_oc_slow_mode(TIM1, TIM_OC2);
	timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);

	/* Configure OC2. */
	timer_set_oc_polarity_high(TIM1, TIM_OC2);
	timer_set_oc_idle_state_set(TIM1, TIM_OC2);

	/* Configure OC2N. */
	timer_set_oc_polarity_high(TIM1, TIM_OC2N);
	timer_set_oc_idle_state_set(TIM1, TIM_OC2N);

	/* Set the capture compare value for OC1. */
	timer_set_oc_value(TIM1, TIM_OC2, 100);

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC2);
	timer_enable_oc_output(TIM1, TIM_OC2N);

	/* -- OC3 and OC3N configuration -- */

	/* Disable outputs. */
	timer_disable_oc_output(TIM1, TIM_OC3);
	timer_disable_oc_output(TIM1, TIM_OC3N);

	/* Configure global mode of line 3. */
	timer_disable_oc_clear(TIM1, TIM_OC3);
	timer_enable_oc_preload(TIM1, TIM_OC3);
	timer_set_oc_slow_mode(TIM1, TIM_OC3);
	timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

	/* Configure OC3. */
	timer_set_oc_polarity_high(TIM1, TIM_OC3);
	timer_set_oc_idle_state_set(TIM1, TIM_OC3);

	/* Configure OC3N. */
	timer_set_oc_polarity_high(TIM1, TIM_OC3N);
	timer_set_oc_idle_state_set(TIM1, TIM_OC3N);

	/* Set the capture compare value for OC3. */
	timer_set_oc_value(TIM1, TIM_OC3, 100);

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC3);
	timer_enable_oc_output(TIM1, TIM_OC3N);

	/* ---- */

	/* ARR reload enable. */
	timer_enable_preload(TIM1);

	/*
	 * Enable preload of complementary channel configurations and
	 * update on COM event.
	 */
	timer_enable_preload_complementry_enable_bits(TIM1);

	/* Enable outputs in the break subsystem. */
	timer_enable_break_main_output(TIM1);

	/* Counter enable. */
	timer_enable_counter(TIM1);

	/* Enable commutation interrupt. */
	timer_enable_irq(TIM1, TIM_DIER_COMIE);

	
}

extern "C"{
void tim1_trg_com_isr(void)
{
	static int step = 0;

	/* Clear the COM trigger interrupt flag. */
	timer_clear_flag(TIM1, TIM_SR_COMIF);

	/*
	 * A simplified and inefficient implementation of PWM On
	 * scheme. Look at the implementation in Open-BLDC on
	 * http://open-bldc.org for the proper implementation. This
	 * one only serves as an example.
	 *
	 * Table of the PWM scheme zone configurations when driving:
	 * @verbatim
	 *  | 1| 2| 3| 4| 5| 6|
	 * -+--+--+--+--+--+--+
	 * A|p+|++|  |p-|--|  |
	 * -+--+--+--+--+--+--+
	 * B|  |p-|--|  |p+|++|
	 * -+--+--+--+--+--+--+
	 * C|--|  |p+|++|  |p-|
	 * -+--+--+--+--+--+--+
	 *  |  |  |  |  |  |  '- 360 Deg
	 *  |  |  |  |  |  '---- 300 Deg
	 *  |  |  |  |  '------- 240 Deg
	 *  |  |  |  '---------- 180 Deg
	 *  |  |  '------------- 120 Deg
	 *  |  '----------------  60 Deg
	 *  '-------------------   0 Deg
	 *
	 * Legend:
	 * p+: PWM on the high side
	 * p-: PWM on the low side
	 * --: Low side on
	 * ++: High side on
	 *   : Floating/NC
	 * @endverbatim
	 */
	switch (step) {
	case 0: /* A PWM HIGH, B OFF, C LOW */
		timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_FROZEN);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_FORCE_LOW);

		timer_enable_oc_output(TIM1, TIM_OC1);
		timer_disable_oc_output(TIM1, TIM_OC1N);

		timer_disable_oc_output(TIM1, TIM_OC2);
		timer_disable_oc_output(TIM1, TIM_OC2N);

		timer_enable_oc_output(TIM1, TIM_OC3);
		timer_enable_oc_output(TIM1, TIM_OC3N);

		step++;
		break;
	case 1: /* A HIGH, B PWM LOW, C OFF */
		timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FORCE_HIGH);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_FROZEN);

		timer_enable_oc_output(TIM1, TIM_OC1);
		timer_enable_oc_output(TIM1, TIM_OC1N);

		timer_disable_oc_output(TIM1, TIM_OC2);
		timer_enable_oc_output(TIM1, TIM_OC2N);

		timer_disable_oc_output(TIM1, TIM_OC3);
		timer_disable_oc_output(TIM1, TIM_OC3N);

		step++;
		break;
	case 2: /* A OFF, B LOW, C PWM HIGH */
		timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FROZEN);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_FORCE_LOW);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

		timer_disable_oc_output(TIM1, TIM_OC1);
		timer_disable_oc_output(TIM1, TIM_OC1N);

		timer_enable_oc_output(TIM1, TIM_OC2);
		timer_enable_oc_output(TIM1, TIM_OC2N);

		timer_enable_oc_output(TIM1, TIM_OC3);
		timer_disable_oc_output(TIM1, TIM_OC3N);

		step++;
		break;
	case 3: /* A PWM LOW, B OFF, C HIGH */
		timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_FROZEN);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_FORCE_HIGH);

		timer_disable_oc_output(TIM1, TIM_OC1);
		timer_enable_oc_output(TIM1, TIM_OC1N);

		timer_disable_oc_output(TIM1, TIM_OC2);
		timer_disable_oc_output(TIM1, TIM_OC2N);

		timer_enable_oc_output(TIM1, TIM_OC3);
		timer_enable_oc_output(TIM1, TIM_OC3N);

		step++;
		break;
	case 4: /* A LOW, B PWM HIGH, C OFF */
		timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FORCE_LOW);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_FROZEN);

		timer_enable_oc_output(TIM1, TIM_OC1);
		timer_enable_oc_output(TIM1, TIM_OC1N);

		timer_enable_oc_output(TIM1, TIM_OC2);
		timer_disable_oc_output(TIM1, TIM_OC2N);

		timer_disable_oc_output(TIM1, TIM_OC3);
		timer_disable_oc_output(TIM1, TIM_OC3N);

		step++;
		break;
	case 5: /* A OFF, B HIGH, C PWM LOW */
		timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FROZEN);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_FORCE_HIGH);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

		timer_disable_oc_output(TIM1, TIM_OC1);
		timer_disable_oc_output(TIM1, TIM_OC1N);

		timer_enable_oc_output(TIM1, TIM_OC2);
		timer_enable_oc_output(TIM1, TIM_OC2N);

		timer_disable_oc_output(TIM1, TIM_OC3);
		timer_enable_oc_output(TIM1, TIM_OC3N);

		step = 0;
		break;
	}

}
}

void adc_setup(void)
{
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

  /* Make sure the ADC doesn't run during config. */
  adc_power_off(ADC1);

  /* We configure everything for one single conversion. */
  adc_disable_scan_mode(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_right_aligned(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

  adc_power_on(ADC1);

  /* Wait for ADC starting up. */
  int i;
  for (i = 0; i < 800000; i++) /* Wait a bit. */
    __asm__("nop");

  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
}

uint16_t read_adc_naiive(uint8_t channel)
{
  uint8_t channel_array[16];
  channel_array[0] = channel;
  adc_set_regular_sequence(ADC1, 1, channel_array);
  adc_start_conversion_direct(ADC1);
  while (!adc_eoc(ADC1));
  uint16_t reg16 = adc_read_regular(ADC1);
  return reg16;
}




/* Set STM32 to 24 MHz. */
void clock_setup(void)
{

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_USART1);

   /* Base board frequency, set to 168Mhz */



        rcc_clock_setup_in_hsi_out_24mhz();
  /* clock rate / 168000 to get 1mS interrupt rate */
  systick_set_reload(24000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();

  /* this done last */
  systick_interrupt_enable();

}

void gpio_setup(void)
{


    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL , GPIO1);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO13);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN , GPIO14);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    usart_set_baudrate(USART1, 38400);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);

}

void user_button_enable(void)
{
  /*Enable GPIOA clock */
  //rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
  /*set GPIOA0 as input open-drain */
  //gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO13);
}


int main( void )
{

  clock_setup();
  gpio_setup();
  user_button_enable();
  adc_setup();
  tim_setup();
  
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
		timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_FROZEN);
		timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_FORCE_LOW);

		timer_enable_oc_output(TIM1, TIM_OC1);
		timer_disable_oc_output(TIM1, TIM_OC1N);

		timer_disable_oc_output(TIM1, TIM_OC2);
		timer_disable_oc_output(TIM1, TIM_OC2N);

		timer_enable_oc_output(TIM1, TIM_OC3);
		timer_enable_oc_output(TIM1, TIM_OC3N);

  char buffer[64];

  std::string s;

    while(1)
    {


        gpio_toggle(GPIOB, GPIO1);

        uint16_t adcValue = read_adc_naiive(0);
        s = std::to_string(adcValue);

        printf("%d\n",adcValue);



        msleep(1000);
    }

    goDown();

    while(true)
    {
        switch (state)
        {
            case STATE::INIT:
                init();
                break;

            case STATE::GO_UP:
                goUp();
                break;

            case STATE::GO_DOWN:
                goDown();
                break;

        }
    }

  return 0;
}
