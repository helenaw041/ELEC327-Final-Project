/**
 * @file    phased_pwm.c
 * @brief   4-channel phase-shifted PWM on TIMA0 with ISR-triggered DMA CCR reloads.
 *          Designed for MSPM0G3507 ultrasonic phased array.
 *
 * Pin Mapping:
 *   PA8  (PINCM19, func 5) -> TIMA0_C0  (Channel 0)
 *   PA9  (PINCM20, func 5) -> TIMA0_C1  (Channel 1)
 *   PA7  (PINCM14, func 5) -> TIMA0_C2  (Channel 2)
 *   PA12 (PINCM34, func 6) -> TIMA0_C3  (Channel 3)
 *
 * Architecture:
 *   TIMA0 counts 0 -> ARR (799) at 32 MHz = 40 kHz period (25 us).
 *
 *   Each channel needs a rising edge at phase_ticks[n] and a falling
 *   edge at (phase_ticks[n] + duty_ticks) % PERIOD.
 *
 *   g_ccr_table[] holds precomputed CC values for all channels:
 *     g_ccr_table[0..1] = CC_01[0], CC_01[1]  (CH0, CH1 rise ticks)
 *     g_ccr_table[2..3] = CC_23[0], CC_23[1]  (CH2, CH3 rise ticks)
 *     g_ccr_table[4..5] = CC_01[0], CC_01[1]  (CH0, CH1 fall ticks)
 *     g_ccr_table[6..7] = CC_23[0], CC_23[1]  (CH2, CH3 fall ticks)
 *
 *   ISR flow (runs every 25 us):
 *     ZERO event  -> software-trigger DMA CH0: writes rise ticks to CC_01, CC_23
 *     CC0  event  -> software-trigger DMA CH1: writes fall ticks to CC_01, CC_23
 *
 *   The CCACT registers are configured so:
 *     On CC match during up-count: pin goes HIGH  (rise)
 *     On CC match during up-count after DMA CH1 loads fall ticks: pin goes LOW
 *
 *   Phase updates from main() write to g_ccr_table[] directly.
 *   DMA picks up new values on the very next period — no timer restart needed.
 *
 * @note Register names verified against mspm0g350x.h build date 2020-08-28.
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
#define PWM_FREQ_HZ             40000UL
#define TIMA0_PRESCALER         1U
#define TIMA0_ARR               ((uint32_t)((BUSCLK_HZ / (TIMA0_PRESCALER * PWM_FREQ_HZ)) - 1U))  /* 799 */
#define TICKS_PER_PERIOD        (TIMA0_ARR + 1U)   /* 800 */
#define DEFAULT_DUTY_PCT        50U
#define NUM_CH                  4U

/* =========================================================================
 * CCR Table (DMA source)
 *
 * Layout matches the CC_01[] and CC_23[] register arrays exactly so DMA
 * can do a 2-word burst directly into each register pair.
 *
 * Rise table  (g_ccr_table[0..3]): loaded at ZERO event
 *   [0] -> CC_01[0]  (CH0 compare value)
 *   [1] -> CC_01[1]  (CH1 compare value)
 *   [2] -> CC_23[0]  (CH2 compare value)
 *   [3] -> CC_23[1]  (CH3 compare value)
 *
 * Fall table  (g_ccr_table[4..7]): loaded at CC0 match event
 *   [4] -> CC_01[0]
 *   [5] -> CC_01[1]
 *   [6] -> CC_23[0]
 *   [7] -> CC_23[1]
 * ========================================================================= */

static volatile uint32_t g_ccr_table[8];

static volatile float   g_phase_deg[NUM_CH] = {0.0f, 90.0f, 180.0f, 270.0f};
static volatile uint8_t g_duty_pct = DEFAULT_DUTY_PCT;

/* =========================================================================
 * DMA state: track which transfer to trigger next in the ISR
 * ========================================================================= */

typedef enum {
    DMA_STAGE_RISE = 0,
    DMA_STAGE_FALL = 1
} DMAStage_t;

static volatile DMAStage_t g_dma_stage = DMA_STAGE_RISE;

/* =========================================================================
 * Math helpers
 * ========================================================================= */

static inline uint32_t degrees_to_ticks(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return (uint32_t)((deg / 360.0f) * (float)TICKS_PER_PERIOD);
}

/**
 * @brief Recompute g_ccr_table from current phase/duty settings.
 *        Safe to call from main(). DMA picks up new values next period.
 */
static void recalculate_ccr_table(void)
{

    uint32_t duty_ticks = (TICKS_PER_PERIOD * (uint32_t)g_duty_pct) / 100U;

    for (uint32_t ch = 0; ch < NUM_CH; ch++)
    {
        uint32_t rise = degrees_to_ticks(g_phase_deg[ch]);
        uint32_t fall = (rise + duty_ticks) % TICKS_PER_PERIOD;

        if (rise == 0U) rise = 1U;
        if (fall == 0U) fall = 1U;

        g_ccr_table[ch]          = rise;
        g_ccr_table[ch + NUM_CH] = fall;
    }
}

/* =========================================================================
 * GPIO Initialization
 * ========================================================================= */

static void GPIO_init(void)
{
    GPIOA->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W       |
                            GPIO_RSTCTL_RESETSTKYCLR_CLR   |
                            GPIO_RSTCTL_RESETASSERT_ASSERT);

    GPIOA->GPRCM.PWREN  = (GPIO_PWREN_KEY_UNLOCK_W |
                            GPIO_PWREN_ENABLE_ENABLE);

    delay_cycles(POWER_STARTUP_DELAY);

    /* PA8  PINCM19 func 5 -> TIMA0_C0 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM19] = (IOMUX_PINCM_PC_CONNECTED | 0x00000005U);
    /* PA9  PINCM20 func 5 -> TIMA0_C1 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM20] = (IOMUX_PINCM_PC_CONNECTED | 0x00000005U);
    /* PA7  PINCM14 func 5 -> TIMA0_C2 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM14] = (IOMUX_PINCM_PC_CONNECTED | 0x00000005U);
    /* PA12 PINCM34 func 6 -> TIMA0_C3 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM34] = (IOMUX_PINCM_PC_CONNECTED | 0x00000006U);
}

/* =========================================================================
 * TIMA0 Initialization
 * ========================================================================= */

/*
 * CCACT register bit fields (from TRM, verified against header offsets).
 * Each CCACT_01[n] / CCACT_23[n] register controls two channels.
 *
 * Bits [3:0]  = CC0/CC2 action on up-count match (CUACTx)
 * Bits [7:4]  = CC0/CC2 action on down-count match
 * Bits [19:16]= CC1/CC3 action on up-count match (CUACTx)
 * Bits [23:20]= CC1/CC3 action on down-count match
 *
 * Action values (4-bit field):
 *   0x0 = disabled
 *   0x1 = CCP HIGH (set pin high on match)
 *   0x2 = CCP LOW  (set pin low on match)
 *   0x3 = toggle
 */
#define CCACT_CUACT_HIGH        0x1U   /* set HIGH on up-count match */
#define CCACT_CUACT_LOW         0x2U   /* set LOW  on up-count match */

/* Bit offsets within CCACT_01[]/CCACT_23[] */
#define CCACT_CC0_CUACT_OFS     0U
#define CCACT_CC1_CUACT_OFS     16U

static void TIMA0_init(void)
{
    TIMA0->GPRCM.RSTCTL = (GPTIMER_RSTCTL_KEY_UNLOCK_W       |
                            GPTIMER_RSTCTL_RESETSTKYCLR_CLR   |
                            GPTIMER_RSTCTL_RESETASSERT_ASSERT);

    TIMA0->GPRCM.PWREN  = (GPTIMER_PWREN_KEY_UNLOCK_W |
                            GPTIMER_PWREN_ENABLE_ENABLE);

    delay_cycles(POWER_STARTUP_DELAY);

    /* Clock: BUSCLK, divide by 1 */
    TIMA0->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    TIMA0->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_1;

    /* Up-count, continuous repeat, counter starts at 0 */
    TIMA0->COUNTERREGS.CTRCTL =
        GPTIMER_CTRCTL_CVAE_ZEROVAL   |
        GPTIMER_CTRCTL_CM_UP          |
        GPTIMER_CTRCTL_REPEAT_REPEAT_1;

    /* Period: 800 ticks = 40 kHz */
    TIMA0->COUNTERREGS.LOAD = TIMA0_ARR;

    /* ---- Initial CCR values (rise ticks) ---- */
    recalculate_ccr_table();

    TIMA0->COUNTERREGS.CC_01[0] = g_ccr_table[0];  /* CH0 */
    TIMA0->COUNTERREGS.CC_01[1] = g_ccr_table[1];  /* CH1 */
    TIMA0->COUNTERREGS.CC_23[0] = g_ccr_table[2];  /* CH2 */
    TIMA0->COUNTERREGS.CC_23[1] = g_ccr_table[3];  /* CH3 */

    TIMA0->COMMONREGS.CCPD =
        GPTIMER_CCPD_C0CCP0_OUTPUT |
        GPTIMER_CCPD_C0CCP1_OUTPUT |
        GPTIMER_CCPD_C0CCP2_OUTPUT |
        GPTIMER_CCPD_C0CCP3_OUTPUT;

    TIMA0->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED;
    /*
     * CCACT: configure output action on compare match.
     *
     * Initial state: SET HIGH on match (rise stage).
     * The ISR will switch this to SET LOW before loading fall ticks,
     * then back to SET HIGH before loading rise ticks next period.
     *
     * CCACT_01[0] controls CH0 and CH1:
     *   bits [3:0]   = CH0 up-count action
     *   bits [19:16] = CH1 up-count action
     */
    TIMA0->COUNTERREGS.CCACT_01[0] =
        (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) |
        (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);

    TIMA0->COUNTERREGS.CCACT_23[0] =
        (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) |
        (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);

    /*
     * Enable CPU interrupts for ZERO event and CC0 match event.
     * ZERO  -> trigger DMA to load rise ticks
     * CC0   -> trigger DMA to load fall ticks
     */
    TIMA0->CPU_INT.IMASK =
        GPTIMER_CPU_INT_IMASK_Z_SET    |   /* ZERO event        */
        GPTIMER_CPU_INT_IMASK_CCU0_SET;    /* CC0 up-count match */

    NVIC_SetPriority(TIMA0_INT_IRQn, 0);
    NVIC_EnableIRQ(TIMA0_INT_IRQn);
}

/* =========================================================================
 * DMA Initialization
 *
 * Two DMA channels, each does a 2-burst block transfer:
 *
 *   DMA CHAN 0: rise ticks -> CC_01[0..1]  (2 words)
 *               rise ticks -> CC_23[0..1]  (2 words)
 *   DMA CHAN 1: fall ticks -> CC_01[0..1]
 *               fall ticks -> CC_23[0..1]
 *
 * Both are software-triggered from the ISR (TIMA0 has no DMA trigger line
 * in this SDK version). The ISR writes DMATCTL = DMA_SOFTWARE_TRIG to fire.
 *
 * Each DMA channel is configured to transfer 2 words (CC_01 pair), then
 * the ISR fires a second transfer for CC_23. Or we chain two channels per
 * stage. For simplicity we use 4 DMA channels total:
 *   CH0: rise -> CC_01
 *   CH1: rise -> CC_23
 *   CH2: fall -> CC_01
 *   CH3: fall -> CC_23
 *
 * The ISR triggers CH0+CH1 on ZERO event, CH2+CH3 on CC0 event.
 * ========================================================================= */

/*
 * DMACTL register bit fields (from struct — single uint32_t per channel).
 * Bit definitions from TRM section on DMA:
 */
#define DMACTL_DMASRCINCR_OFS   26U
#define DMACTL_DMADSTINCR_OFS   24U
#define DMACTL_DMADSTINCR_UNCHANGED  (0U << DMACTL_DMADSTINCR_OFS)
#define DMACTL_DMADSTINCR_INC1       (1U << DMACTL_DMADSTINCR_OFS)  /* +2 bytes */
#define DMACTL_DMADSTINCR_INC2       (2U << DMACTL_DMADSTINCR_OFS)  /* +4 bytes (word) */
#define DMACTL_DMASRCINCR_UNCHANGED  (0U << DMACTL_DMASRCINCR_OFS)
#define DMACTL_DMASRCINCR_INC1       (1U << DMACTL_DMASRCINCR_OFS)
#define DMACTL_DMASRCINCR_INC2       (2U << DMACTL_DMASRCINCR_OFS)  /* +4 bytes (word) */
#define DMACTL_DMADSTWIDTH_OFS  20U
#define DMACTL_DMASRCWIDTH_OFS  18U
#define DMACTL_DMADSTWIDTH_WORD (2U << DMACTL_DMADSTWIDTH_OFS)
#define DMACTL_DMASRCWIDTH_WORD (2U << DMACTL_DMASRCWIDTH_OFS)
#define DMACTL_DMAMODE_OFS      14U
#define DMACTL_DMAMODE_BLOCK    (1U << DMACTL_DMAMODE_OFS)
#define DMACTL_DMAEN_OFS        0U
#define DMACTL_DMAEN            (1U << DMACTL_DMAEN_OFS)

static void DMA_init(void)
{
    /* DMA has no GPRCM in this header — power is always on */

    /*
     * Configure 4 DMA channels:
     *   CH0: g_ccr_table[0..1] -> TIMA0 CC_01[0..1]   (rise, pair 01)
     *   CH1: g_ccr_table[2..3] -> TIMA0 CC_23[0..1]   (rise, pair 23)
     *   CH2: g_ccr_table[4..5] -> TIMA0 CC_01[0..1]   (fall, pair 01)
     *   CH3: g_ccr_table[6..7] -> TIMA0 CC_23[0..1]   (fall, pair 23)
     *
     * All channels: software trigger, 2-word block, word width,
     *               src increment, dst increment, re-armed after transfer.
     */

    uint32_t ctl = DMACTL_DMASRCINCR_INC2     |   /* src +4 bytes per word */
                   DMACTL_DMADSTINCR_INC2     |   /* dst +4 bytes per word */
                   DMACTL_DMASRCWIDTH_WORD    |   /* 32-bit source          */
                   DMACTL_DMADSTWIDTH_WORD    |   /* 32-bit destination     */
                   DMACTL_DMAMODE_BLOCK       |   /* block transfer         */
                   DMACTL_DMAEN;                  /* enable                 */

    /* CH0: rise ticks -> CC_01 */
    DMA->DMACHAN[0].DMACTL  = ctl;
    DMA->DMACHAN[0].DMASA   = (uint32_t)&g_ccr_table[0];
    DMA->DMACHAN[0].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_01[0];
    DMA->DMACHAN[0].DMASZ   = 2U;   /* 2 words */
    DMA->DMATRIG[0].DMATCTL = DMA_SOFTWARE_TRIG;

    /* CH1: rise ticks -> CC_23 */
    DMA->DMACHAN[1].DMACTL  = ctl;
    DMA->DMACHAN[1].DMASA   = (uint32_t)&g_ccr_table[2];
    DMA->DMACHAN[1].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_23[0];
    DMA->DMACHAN[1].DMASZ   = 2U;
    DMA->DMATRIG[1].DMATCTL = DMA_SOFTWARE_TRIG;

    /* CH2: fall ticks -> CC_01 */
    DMA->DMACHAN[2].DMACTL  = ctl;
    DMA->DMACHAN[2].DMASA   = (uint32_t)&g_ccr_table[4];
    DMA->DMACHAN[2].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_01[0];
    DMA->DMACHAN[2].DMASZ   = 2U;
    DMA->DMATRIG[2].DMATCTL = DMA_SOFTWARE_TRIG;

    /* CH3: fall ticks -> CC_23 */
    DMA->DMACHAN[3].DMACTL  = ctl;
    DMA->DMACHAN[3].DMASA   = (uint32_t)&g_ccr_table[6];
    DMA->DMACHAN[3].DMADA   = (uint32_t)&TIMA0->COUNTERREGS.CC_23[0];
    DMA->DMACHAN[3].DMASZ   = 2U;
    DMA->DMATRIG[3].DMATCTL = DMA_SOFTWARE_TRIG;
}

/* =========================================================================
 * TIMA0 ISR
 *
 * Kept as short as possible — just switch CCACT and software-trigger DMA.
 * All heavy lifting (memory writes) done by DMA, not the CPU.
 * ========================================================================= */

void TIMA0_IRQHandler(void)
{
    uint32_t isr = TIMA0->CPU_INT.MIS;

    if (isr & GPTIMER_CPU_INT_MIS_Z_SET)
    {
        /* Start of new period: switch CCACT to HIGH, trigger rise DMA */
        // TIMA0->COUNTERREGS.CCACT_01[0] =
        //     (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) |
        //     (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);
        // TIMA0->COUNTERREGS.CCACT_23[0] =
        //     (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) |
        //     (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);
        
        TIMA0->COUNTERREGS.CCACT_01[0] = (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);
        TIMA0->COUNTERREGS.CCACT_01[1] = (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);
        TIMA0->COUNTERREGS.CCACT_23[0] = (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);
        TIMA0->COUNTERREGS.CCACT_23[1] = (CCACT_CUACT_HIGH << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_HIGH << CCACT_CC1_CUACT_OFS);

        /* Software-trigger DMA CH0 and CH1 (rise ticks -> CC_01, CC_23) */
        DMA->DMATRIG[0].DMATCTL = DMA_SOFTWARE_TRIG;
        DMA->DMATRIG[1].DMATCTL = DMA_SOFTWARE_TRIG;

        TIMA0->CPU_INT.ICLR = GPTIMER_CPU_INT_ICLR_Z_CLR;
    }

    if (isr & GPTIMER_CPU_INT_MIS_CCU0_SET)
    {
        /* CH0 just went HIGH: switch CCACT to LOW, trigger fall DMA */
        // TIMA0->COUNTERREGS.CCACT_01[0] =
        //     (CCACT_CUACT_LOW << CCACT_CC0_CUACT_OFS) |
        //     (CCACT_CUACT_LOW << CCACT_CC1_CUACT_OFS);
        // TIMA0->COUNTERREGS.CCACT_23[0] =
        //     (CCACT_CUACT_LOW << CCACT_CC0_CUACT_OFS) |
        //     (CCACT_CUACT_LOW << CCACT_CC1_CUACT_OFS);

        TIMA0->COUNTERREGS.CCACT_01[0] = (CCACT_CUACT_LOW << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_LOW << CCACT_CC1_CUACT_OFS);
        TIMA0->COUNTERREGS.CCACT_01[1] = (CCACT_CUACT_LOW << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_LOW << CCACT_CC1_CUACT_OFS);
        TIMA0->COUNTERREGS.CCACT_23[0] = (CCACT_CUACT_LOW << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_LOW << CCACT_CC1_CUACT_OFS);
        TIMA0->COUNTERREGS.CCACT_23[1] = (CCACT_CUACT_LOW << CCACT_CC0_CUACT_OFS) | (CCACT_CUACT_LOW << CCACT_CC1_CUACT_OFS);

        /* Software-trigger DMA CH2 and CH3 (fall ticks -> CC_01, CC_23) */
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
    recalculate_ccr_table();
}

/**
 * @brief Set phases for all channels atomically.
 * @param phases Array of 4 phase values in degrees.
 * Prefer this over four individual SetPhase() calls.
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
    recalculate_ccr_table();
}

/**
 * @brief Set duty cycle for all channels.
 * @param duty_pct 1..99 (50 recommended for ultrasonic transducers)
 */
void PhaseArray_SetDuty(uint8_t duty_pct)
{
    if (duty_pct == 0U || duty_pct >= 100U) return;
    g_duty_pct = duty_pct;
    recalculate_ccr_table();
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

    /* Switch pins back to GPIO and drive LOW */
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
 *   Speed of sound: c = 343 m/s
 *   Frequency:      f = 40,000 Hz  ->  lambda = 8.575 mm
 *   Element spacing: d = lambda/2 = 4.29 mm
 *   Inter-element phase shift for angle theta:
 *     delta_phi (deg) = 180 * sin(theta)
 *
 *   theta =  0 deg  ->  delta_phi =   0 deg  (broadside)
 *   theta = 30 deg  ->  delta_phi =  90 deg
 *   theta = 90 deg  ->  delta_phi = 180 deg  (endfire)
 * ========================================================================= */

int main(void)
{
    /* Configure system clock for 32 MHz BUSCLK before calling init */
    /* SYSCFG_DL_init(); */

    PhaseArray_Init();

    /* Default: 0, 90, 180, 270 degrees */
    float phases[NUM_CH] = {0.0f, 90.0f, 180.0f, 270.0f};
    PhaseArray_SetAllPhases(phases);

    PhaseArray_Start();

    while (1)
    {
        /* Update phases from main as needed, e.g.: */
        /* PhaseArray_SetPhase(1, 45.0f); */
        /* PhaseArray_SetAllPhases(phases); */
        __WFI();
    }
}