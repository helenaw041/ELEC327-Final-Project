/**
 * @file    phased_pwm.c
 * @brief   4-channel truly phase-shifted 40 kHz PWM on TIMA0 with ISR+DMA.
 *
 * Based on confirmed-working code (CCPD, CCLKCTL, DIV_BY_4, SDK macros).
 *
 * True phase shift: each channel has an independent rising edge at
 * phase_ticks[n] and falls DUTY_TICKS later. All channels at 50% duty.
 *
 * Mechanism:
 *   ZERO event ISR:  switch CCACT to CUACT=HIGH, DMA loads rise ticks
 *   CCU0 event ISR:  switch CCACT to CUACT=LOW,  DMA loads fall ticks
 *   Hardware fires pin transition at CC match — no GPIO writes needed.
 *
 * Pin Mapping:
 *   PA8  (PINCM19, func 5) -> TIMA0_C0
 *   PA9  (PINCM20, func 5) -> TIMA0_C1
 *   PA7  (PINCM14, func 5) -> TIMA0_C2
 *   PA12 (PINCM34, func 6) -> TIMA0_C3
 *
 * Clock:
 *   BUSCLK = 32 MHz, CLKDIV = /4 -> 8 MHz timer clock
 *   LOAD = 199 -> period = 200 ticks = 40 kHz
 *   1 tick = 125 ns = 1.8 degrees of phase
 *
 * @device  MSPM0G3507
 */

#include <ti/devices/msp/msp.h>
#include <stdint.h>
#include <stdbool.h>
#include "delay.h"

/* =========================================================================
 * Configuration
 * ========================================================================= */

#define POWER_STARTUP_DELAY     16U

#define BUSCLK_HZ               32000000UL
#define TIMER_CLK_HZ            (BUSCLK_HZ / 4U)                          /* 8 MHz  */
#define PWM_FREQ_HZ             40000UL
#define PERIOD_TICKS            ((uint32_t)(TIMER_CLK_HZ / PWM_FREQ_HZ))  /* 200    */
#define ARR                     (PERIOD_TICKS - 1U)                        /* 199    */
#define DUTY_TICKS              (PERIOD_TICKS / 2U)                        /* 100    */

#define NUM_CH                  4U

/* =========================================================================
 * CCR Tables (DMA source)
 *
 * g_rise_table[0..3]: rise ticks CH0..CH3  -> loaded at ZERO event
 *   [0] -> CC_01[0]  CH0
 *   [1] -> CC_01[1]  CH1
 *   [2] -> CC_23[0]  CH2
 *   [3] -> CC_23[1]  CH3
 *
 * g_fall_table[0..3]: fall ticks CH0..CH3  -> loaded at CCU0 event
 *   same layout
 * ========================================================================= */

static volatile uint32_t g_rise_table[4];
static volatile uint32_t g_fall_table[4];

static volatile float g_phase_deg[NUM_CH] = {0.0f, 90.0f, 180.0f, 270.0f};

/* =========================================================================
 * Math helpers
 * ========================================================================= */

static inline uint32_t degrees_to_ticks(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return (uint32_t)((deg / 360.0f) * (float)PERIOD_TICKS);
}

/**
 * @brief Recompute rise and fall tables from current phase settings.
 *        rise[n] = phase_ticks[n]  (clamped to min 1)
 *        fall[n] = (rise[n] + DUTY_TICKS) % PERIOD_TICKS  (clamped to min 1)
 *        Safe to call from main(). DMA picks up next period automatically.
 */
static void recalculate_tables(void)
{
    for (uint32_t ch = 0; ch < NUM_CH; ch++)
    {
        uint32_t rise = degrees_to_ticks(g_phase_deg[ch]);
        uint32_t fall = (rise + DUTY_TICKS) % PERIOD_TICKS;

        /* CC=0 fires at the same instant as ZERO event and won't generate
         * a CCU interrupt on up-count, so clamp both to minimum 1 tick. */
        if (rise == 0U) rise = 1U;
        if (fall == 0U) fall = 1U;

        g_rise_table[ch] = rise;
        g_fall_table[ch] = fall;
    }
}

/* =========================================================================
 * DMA bit field definitions
 * ========================================================================= */

#define DMACTL_DMASRCINCR_OFS        26U
#define DMACTL_DMADSTINCR_OFS        24U
#define DMACTL_DMADSTINCR_INC2       (2U << DMACTL_DMADSTINCR_OFS)
#define DMACTL_DMASRCINCR_INC2       (2U << DMACTL_DMASRCINCR_OFS)
#define DMACTL_DMADSTWIDTH_OFS       20U
#define DMACTL_DMASRCWIDTH_OFS       18U
#define DMACTL_DMADSTWIDTH_WORD      (2U << DMACTL_DMADSTWIDTH_OFS)
#define DMACTL_DMASRCWIDTH_WORD      (2U << DMACTL_DMASRCWIDTH_OFS)
#define DMACTL_DMAMODE_OFS           14U
#define DMACTL_DMAMODE_BLOCK         (1U << DMACTL_DMAMODE_OFS)
#define DMACTL_DMAEN                 (1U << 0U)

/* =========================================================================
 * GPIO Initialization — identical to working code
 * ========================================================================= */

static void GPIO_init(void)
{
    GPIOA->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W       |
                            GPIO_RSTCTL_RESETSTKYCLR_CLR   |
                            GPIO_RSTCTL_RESETASSERT_ASSERT);

    GPIOA->GPRCM.PWREN  = (GPIO_PWREN_KEY_UNLOCK_W |
                            GPIO_PWREN_ENABLE_ENABLE);

    delay_cycles(POWER_STARTUP_DELAY);

    IOMUX->SECCFG.PINCM[IOMUX_PINCM19] = (IOMUX_PINCM_PC_CONNECTED | 0x00000005U); /* PA8  C0 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM20] = (IOMUX_PINCM_PC_CONNECTED | 0x00000005U); /* PA9  C1 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM14] = (IOMUX_PINCM_PC_CONNECTED | 0x00000005U); /* PA7  C2 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM34] = (IOMUX_PINCM_PC_CONNECTED | 0x00000006U); /* PA12 C3 */
}

/* =========================================================================
 * TIMA0 Initialization
 *
 * Same structure as working code EXCEPT:
 *   - CCACT uses CUACT=HIGH initially (pin goes HIGH on CC match)
 *     instead of ZACT=HIGH/CUACT=LOW
 *   - Both ZERO and CCU0 interrupts enabled
 *   - 4 DMA channels instead of 2
 * ========================================================================= */

static void TIMA0_init(void)
{
    TIMA0->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W       |
                            GPTIMER_RSTCTL_RESETSTKYCLR_CLR   |
                            GPTIMER_RSTCTL_RESETASSERT_ASSERT);

    TIMA0->GPRCM.PWREN  = (GPTIMER_PWREN_KEY_UNLOCK_W |
                            GPTIMER_PWREN_ENABLE_ENABLE);

    delay_cycles(POWER_STARTUP_DELAY);

    /* 32 MHz / 4 = 8 MHz — same as working code */
    TIMA0->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    TIMA0->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_4;

    /* Up-count, continuous, start at 0, disabled until PhaseArray_Start() */
    TIMA0->COUNTERREGS.CTRCTL =
        GPTIMER_CTRCTL_REPEAT_REPEAT_1 |
        GPTIMER_CTRCTL_CM_UP           |
        GPTIMER_CTRCTL_CVAE_ZEROVAL    |
        GPTIMER_CTRCTL_EN_DISABLED;

    /* 200 ticks = 40 kHz */
    TIMA0->COUNTERREGS.LOAD = ARR;

    /*
     * CCACT: CUACT=HIGH — pin goes HIGH when counter matches CC value.
     * ISR will switch to CUACT=LOW after loading fall ticks, and back
     * to CUACT=HIGH after loading rise ticks at the next ZERO event.
     *
     * Note: ZACT is not set here (no action at counter zero) because
     * we want each channel to rise independently at its phase tick,
     * not all simultaneously at tick 0.
     */
    TIMA0->COUNTERREGS.CCACT_01[0] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;
    TIMA0->COUNTERREGS.CCACT_01[1] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;
    TIMA0->COUNTERREGS.CCACT_23[0] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;
    TIMA0->COUNTERREGS.CCACT_23[1] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;

    /* Load initial rise ticks */
    recalculate_tables();

    TIMA0->COUNTERREGS.CC_01[0] = g_rise_table[0];
    TIMA0->COUNTERREGS.CC_01[1] = g_rise_table[1];
    TIMA0->COUNTERREGS.CC_23[0] = g_rise_table[2];
    TIMA0->COUNTERREGS.CC_23[1] = g_rise_table[3];

    /* Enable all 4 CC pins as outputs — from working code */
    TIMA0->COMMONREGS.CCPD =
        GPTIMER_CCPD_C0CCP0_OUTPUT |
        GPTIMER_CCPD_C0CCP1_OUTPUT |
        GPTIMER_CCPD_C0CCP2_OUTPUT |
        GPTIMER_CCPD_C0CCP3_OUTPUT;

    /* Open clock gate — from working code */
    TIMA0->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED;

    /* Enable ZERO and CCU0 interrupts */
    TIMA0->CPU_INT.IMASK =
        GPTIMER_CPU_INT_IMASK_Z_SET    |
        GPTIMER_CPU_INT_IMASK_CCU0_SET;

    NVIC_SetPriority(TIMA0_INT_IRQn, 0);
    NVIC_EnableIRQ(TIMA0_INT_IRQn);
}

/* =========================================================================
 * DMA Initialization
 *
 * 4 channels:
 *   CH0: g_rise_table[0..1] -> CC_01[0..1]  (CH0+CH1 rise, ZERO trigger)
 *   CH1: g_rise_table[2..3] -> CC_23[0..1]  (CH2+CH3 rise, ZERO trigger)
 *   CH2: g_fall_table[0..1] -> CC_01[0..1]  (CH0+CH1 fall, CCU0 trigger)
 *   CH3: g_fall_table[2..3] -> CC_23[0..1]  (CH2+CH3 fall, CCU0 trigger)
 * ========================================================================= */

static void DMA_init(void)
{
    uint32_t ctl = DMACTL_DMASRCINCR_INC2   |
                   DMACTL_DMADSTINCR_INC2   |
                   DMACTL_DMASRCWIDTH_WORD  |
                   DMACTL_DMADSTWIDTH_WORD  |
                   DMACTL_DMAMODE_BLOCK     |
                   DMACTL_DMAEN;

    /* CH0: rise ticks CH0,CH1 -> CC_01 */
    DMA->DMACHAN[0].DMACTL  = ctl;
    DMA->DMACHAN[0].DMASA   = (uint32_t)&g_rise_table[0];
    DMA->DMACHAN[0].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_01[0];
    DMA->DMACHAN[0].DMASZ   = 2U;
    DMA->DMATRIG[0].DMATCTL = DMA_SOFTWARE_TRIG;

    /* CH1: rise ticks CH2,CH3 -> CC_23 */
    DMA->DMACHAN[1].DMACTL  = ctl;
    DMA->DMACHAN[1].DMASA   = (uint32_t)&g_rise_table[2];
    DMA->DMACHAN[1].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_23[0];
    DMA->DMACHAN[1].DMASZ   = 2U;
    DMA->DMATRIG[1].DMATCTL = DMA_SOFTWARE_TRIG;

    /* CH2: fall ticks CH0,CH1 -> CC_01 */
    DMA->DMACHAN[2].DMACTL  = ctl;
    DMA->DMACHAN[2].DMASA   = (uint32_t)&g_fall_table[0];
    DMA->DMACHAN[2].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_01[0];
    DMA->DMACHAN[2].DMASZ   = 2U;
    DMA->DMATRIG[2].DMATCTL = DMA_SOFTWARE_TRIG;

    /* CH3: fall ticks CH2,CH3 -> CC_23 */
    DMA->DMACHAN[3].DMACTL  = ctl;
    DMA->DMACHAN[3].DMASA   = (uint32_t)&g_fall_table[2];
    DMA->DMACHAN[3].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_23[0];
    DMA->DMACHAN[3].DMASZ   = 2U;
    DMA->DMATRIG[3].DMATCTL = DMA_SOFTWARE_TRIG;
}

/* =========================================================================
 * TIMA0 ISR
 *
 * ZERO event (every 25 us):
 *   1. Switch CCACT to CUACT=HIGH (pin goes HIGH on next CC match)
 *   2. DMA loads rise ticks into CC registers
 *   -> Each pin rises independently when counter hits rise_tick[n]
 *
 * CCU0 event (when CH0 rises = first channel match on up-count):
 *   1. Switch CCACT to CUACT=LOW (pin goes LOW on next CC match)
 *   2. DMA loads fall ticks into CC registers
 *   -> Each pin falls independently when counter hits fall_tick[n]
 * ========================================================================= */

void TIMA0_IRQHandler(void)
{
    uint32_t isr = TIMA0->CPU_INT.MIS;

    if (isr & GPTIMER_CPU_INT_MIS_Z_SET)
    {
        /* Switch all channels to SET HIGH on next match */
        TIMA0->COUNTERREGS.CCACT_01[0] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;
        TIMA0->COUNTERREGS.CCACT_01[1] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;
        TIMA0->COUNTERREGS.CCACT_23[0] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;
        TIMA0->COUNTERREGS.CCACT_23[1] = GPTIMER_CCACT_01_CUACT_CCP_HIGH;

        /* DMA loads rise ticks -> CC_01 and CC_23 */
        DMA->DMATRIG[0].DMATCTL = DMA_SOFTWARE_TRIG;
        DMA->DMATRIG[1].DMATCTL = DMA_SOFTWARE_TRIG;

        TIMA0->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;
    }

    if (isr & GPTIMER_CPU_INT_MIS_CCU0_SET)
    {
        /* Switch all channels to SET LOW on next match */
        TIMA0->COUNTERREGS.CCACT_01[0] = GPTIMER_CCACT_01_CUACT_CCP_LOW;
        TIMA0->COUNTERREGS.CCACT_01[1] = GPTIMER_CCACT_01_CUACT_CCP_LOW;
        TIMA0->COUNTERREGS.CCACT_23[0] = GPTIMER_CCACT_01_CUACT_CCP_LOW;
        TIMA0->COUNTERREGS.CCACT_23[1] = GPTIMER_CCACT_01_CUACT_CCP_LOW;

        /* DMA loads fall ticks -> CC_01 and CC_23 */
        DMA->DMATRIG[2].DMATCTL = DMA_SOFTWARE_TRIG;
        DMA->DMATRIG[3].DMATCTL = DMA_SOFTWARE_TRIG;

        TIMA0->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_CCU0_CLR;
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Set phase for a single channel.
 * @param channel   0..3
 * @param phase_deg 0.0 to 359.9 degrees
 * Takes effect at the start of the next PWM period.
 */
void PhaseArray_SetPhase(uint8_t channel, float phase_deg)
{
    if (channel >= NUM_CH) return;
    while (phase_deg >= 360.0f) phase_deg -= 360.0f;
    while (phase_deg <    0.0f) phase_deg += 360.0f;
    g_phase_deg[channel] = phase_deg;
    recalculate_tables();
}

/**
 * @brief Set phases for all channels atomically.
 * @param phases Array of 4 phase values in degrees.
 */
void PhaseArray_SetAllPhases(const float phases[NUM_CH])
{
    for (uint8_t ch = 0; ch < NUM_CH; ch++)
    {
        float deg = phases[ch];
        while (deg >= 360.0f) deg -= 360.0f;
        while (deg <    0.0f) deg += 360.0f;
        g_phase_deg[ch] = deg;
    }
    recalculate_tables();
}

/**
 * @brief Initialize GPIO, TIMA0, and DMA. Call once before Start().
 */
void PhaseArray_Init(void)
{
    GPIO_init();
    TIMA0_init();
    DMA_init();
}

/**
 * @brief Start PWM output on all channels.
 */
void PhaseArray_Start(void)
{
    TIMA0->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
}

/**
 * @brief Stop PWM and drive all pins LOW safely.
 */
void PhaseArray_Stop(void)
{
    TIMA0->COUNTERREGS.CTRCTL &= ~GPTIMER_CTRCTL_EN_ENABLED;

    IOMUX->SECCFG.PINCM[IOMUX_PINCM19] = (IOMUX_PINCM_PC_CONNECTED | 0x00000001U);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM20] = (IOMUX_PINCM_PC_CONNECTED | 0x00000001U);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM14] = (IOMUX_PINCM_PC_CONNECTED | 0x00000001U);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM34] = (IOMUX_PINCM_PC_CONNECTED | 0x00000001U);

    GPIOA->DOUTCLR31_0 = (1U << 8) | (1U << 9) | (1U << 7) | (1U << 12);
    GPIOA->DOESET31_0  = (1U << 8) | (1U << 9) | (1U << 7) | (1U << 12);
}

/* =========================================================================
 * Example main()
 *
 * Beam steering formula (linear array, half-wavelength spacing):
 *   c = 343 m/s, f = 40 kHz, lambda = 8.575 mm, d = lambda/2 = 4.29 mm
 *   delta_phi (deg) = 180 * sin(theta)
 *   theta =  0 deg -> delta_phi =   0 deg (broadside)
 *   theta = 30 deg -> delta_phi =  90 deg
 *   theta = 90 deg -> delta_phi = 180 deg (endfire)
 * ========================================================================= */

int main(void)
{
    PhaseArray_Init();

    /* Broadside: 0, 90, 180, 270 degrees */
    float phases[NUM_CH] = {0.0f, 90.0f, 180.0f, 270.0f};
    PhaseArray_SetAllPhases(phases);

    PhaseArray_Start();

    while (1)
    {
        __WFI();
    }
}
