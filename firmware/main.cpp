/*****************************************************************//**
 * @file main.cpp
 *
 * @brief Proof of concept touch screen controller comprised of software DDFS, ADC sampling scheme, and tone extraction
 *  coordinated by master control loop and state machine.
 *
 * @author Samuel Hussey
 * @version v1.0: initial release
 *********************************************************************/

// #define _DEBUG
#include "main.h"
#include "chu_init.h"
#include "gpio_cores.h"
#include "xadc_core.h"
#include "sseg_core.h"
#include "ddfs_core.h"
#include "math.h"

// ADC buffers
static uint16_t adc_buf[NUM_RX][ADC_NUM_SAMPLES];
static float adc_res[NUM_RX][ADC_NUM_SAMPLES];

// the duty cycle update LUT
static uint16_t dds_LUT[NUM_TX][DDS_UPDATES_PER_FRAME];

// result buffers
static float heatmap[NUM_TX][NUM_RX];
static float heatmap_filtered[NUM_TX][NUM_RX];

// Goertzel constants to be computed at initialization
int k_0, k_1, k_2, k_3;
float w_0, w_1, w_2, w_3;
float sine_0, sine_1, sine_2, sine_3;
float cosine_0, cosine_1, cosine_2, cosine_3;
float coeff_0, coeff_1, coeff_2, coeff_3;

// initial state
state_t curr_state = CAPTURE;

// Frequency control words used to compute dds_LUT
uint8_t fcw_select[NUM_TX] = {10, 12, 14, 16};


/*
 * Function: nco_LUT_init
 * ----------------------------
 *   Initializes the LUT for duty cycle updates.
 *
 *   returns: void
 */
void dds_LUT_init() {
    uint16_t phase_accum; // Determines where duty cycle values fall in the LUT, basically steps
                          // through the sine table at varying increments depending on the FCW

	for (uint16_t i = 0; i < NUM_TX; i++) {
		phase_accum = 0;
		// Get sine table value based on phase accumulator and put it in the LUT
		for (uint16_t j = 0; j < DDS_UPDATES_PER_FRAME; j++) {
			dds_LUT[i][j] = sine_table[(phase_accum & 511U)];

			// Add the phase increment, fcw
			phase_accum += fcw_select[i];
		}
	}
}

/*
 * Function: iir_filter_init
 * ----------------------------
 *   Initializes the IIR LP filter buffer with all ones.
 *
 *   returns: void
 */
void iir_filter_init() {
	for(uint8_t i = 0; i < NUM_TX; i++) {
		for(uint8_t j = 0; j < NUM_RX; j++) {
			heatmap_filtered[i][j] = 1.0f;
		}
	}
}

/*
 * Function: gen_and_sample_tones
 * ----------------------------
 *   Updates each PWM duty cycle then does a read on the indicated adc channel.
 *   The adc samples at 1 Msps but is gated here by the pwm updates, resulting in ~180 ksps.
 *   The sampling and pwm updates are asynchronous so the tone frequencies don't match the
 *   DDFS output equation, Fout = (M * Fsys) / 2**N, but they are consistent enough to get
 *   decent results. Ideally, you want to sample the same phases each frame for better mag data.
 *
 *   pwm_p: pointer to pwm core instance
 *   adc_p: pointer to adc core instance
 *   adc_channel: the channel to read from
 *   led_p: led used for timing measurements
 *
 *   returns: void
 */
void gen_and_sample_tones(PwmCore *pwm_p, XadcCore *adc_p, int adc_channel, GpoCore *led_p) {

	for (uint16_t i = 0; i < DDS_UPDATES_PER_FRAME; i++) {
		for (uint16_t j = 0; j < NUM_TX; j++) {
			pwm_p->set_duty(dds_LUT[j][i], j);
		}

		// This sampling scheme gives a rate of 182.76 ksps
		led_p->write(1, 0);
		adc_buf[adc_channel][i] = adc_p->read_raw(adc_channel);
		led_p->write(0, 0);
	}

	pwm_p->set_duty(0, 0);
	pwm_p->set_duty(0, 1);
	pwm_p->set_duty(0, 2);
	pwm_p->set_duty(0, 3);
}

/*
 * Function: goertzel_init
 * ----------------------------
 *   Precomputing constants to be used by the Goertzel filter.
 *
 *   returns: void
 */
void goertzel_init() {

    k_0 = (int) (0.5 + ((BLOCK_SIZE * TARGET_FREQ_0) / SAMPLING_RATE));
    k_1 = (int) (0.5 + ((BLOCK_SIZE * TARGET_FREQ_1) / SAMPLING_RATE));
    k_2 = (int) (0.5 + ((BLOCK_SIZE * TARGET_FREQ_2) / SAMPLING_RATE));
    k_3 = (int) (0.5 + ((BLOCK_SIZE * TARGET_FREQ_3) / SAMPLING_RATE));

    w_0 = (2.0 * M_PI * k_0) / BLOCK_SIZE;
    w_1 = (2.0 * M_PI * k_1) / BLOCK_SIZE;
    w_2 = (2.0 * M_PI * k_2) / BLOCK_SIZE;
    w_3 = (2.0 * M_PI * k_3) / BLOCK_SIZE;

    sine_0 = sin(w_0);
    sine_1 = sin(w_1);
    sine_2 = sin(w_2);
    sine_3 = sin(w_3);

    cosine_0 = cos(w_0);
    cosine_1 = cos(w_1);
    cosine_2 = cos(w_2);
    cosine_3 = cos(w_3);

    coeff_0 = 2.0 * cosine_0;
    coeff_1 = 2.0 * cosine_1;
    coeff_2 = 2.0 * cosine_2;
    coeff_3 = 2.0 * cosine_3;
}

/*
 * Function: run_goertzel_mag
 * ----------------------------
 *   Walks through a block of ADC samples (doesn't have to be power of 2)
 *   and determines whether a target frequency is present.
 *   An good tutorial I used: https://www.embedded.com/the-goertzel-algorithm/
 *
 *   sine: sine term for retrieving imaginary component of result
 *   cosine: cosine term for retrieving real component of result
 *   coeff: The decay term in the algorithm's built in iir filter
 *   data: a block of ADC samples
 *
 *   returns: squared magnitude of the coefficient of the DFT term indicated by precomputed constants.
 */
float run_goertzel_mag(float sine, float cosine, float coeff, float* data) {

    float q0, q1, q2;
	float magnitude, real, imag;

    q0=0;
    q1=0;
    q2=0;

    for(uint16_t i = 0; i < (int) BLOCK_SIZE; i++) {
        q0 = data[i] + coeff * q1 - q2;
        q2 = q1;
        q1 = q0;
    }

    // calculate the real and imaginary results
    // scaling appropriately
    real = (q1 - q2 * cosine) * GOERTZEL_SCALAR;
    imag = (q2 * sine) * GOERTZEL_SCALAR;

    magnitude = (real*real + imag*imag); //sqrtf(real*real + imag*imag);
    return magnitude;
}

GpoCore led(get_slot_addr(BRIDGE_BASE, S2_LED));
XadcCore adc(get_slot_addr(BRIDGE_BASE, S5_XDAC));
PwmCore pwm(get_slot_addr(BRIDGE_BASE, S6_PWM));

int main() {

   dds_LUT_init();	// precompute duty cycle vals and other constants
   goertzel_init();
   iir_filter_init();
   pwm.set_freq(PWM_FREQ);

   while (1) {

	   switch(curr_state)
	   {
		   case CAPTURE:
			   gen_and_sample_tones(&pwm, &adc, 0, &led);
			   gen_and_sample_tones(&pwm, &adc, 1, &led);
			   gen_and_sample_tones(&pwm, &adc, 2, &led);
			   gen_and_sample_tones(&pwm, &adc, 3, &led);
			   curr_state = COMPUTE_HEATMAP;
			   break;
		   case COMPUTE_HEATMAP:
			   led.write(1, 1);
			   for (uint8_t i = 0; i < NUM_RX; i++) {
				   for (uint16_t j = 0; j < ADC_NUM_SAMPLES; j++) {
					   adc_res[i][j] = (1.0f * (float)adc_buf[i][j]);
				   }

				   heatmap[0][i] = run_goertzel_mag(sine_0, cosine_0, coeff_0, adc_res[i]);
				   heatmap[1][i] = run_goertzel_mag(sine_1, cosine_1, coeff_1, adc_res[i]);
				   heatmap[2][i] = run_goertzel_mag(sine_2, cosine_2, coeff_2, adc_res[i]);
				   heatmap[3][i] = run_goertzel_mag(sine_3, cosine_3, coeff_3, adc_res[i]);
			   }

			   // IIR LP filter
			   for (uint8_t i = 0; i < NUM_TX; i++) {
				   for (uint8_t j = 0; j < NUM_RX; j++) {
					   heatmap_filtered[i][j] += (IIR_DECAY * (heatmap[i][j] - heatmap_filtered[i][j]));
				   }
			   }
			   led.write(0, 1);

			   curr_state = OUTPUT_HEATMAP;
			   break;

		   case OUTPUT_HEATMAP:
			   led.write(1, 2);
			   int val;
			   uart.disp("x\n");
			   for (uint8_t i = 0; i < NUM_TX; i++) {
				   for (uint8_t j = 0; j < NUM_RX; j++) {
					   val = (int) heatmap_filtered[i][j];
					   uart.disp(val);
					   uart.disp("\n");
				   }
			   }
			   uart.disp("z\n");

			   led.write(0, 2);
			   curr_state = CAPTURE;
			   break;
		   default:
			   curr_state = CAPTURE;
			   break;

	   } //switch
   } //while
} //main

