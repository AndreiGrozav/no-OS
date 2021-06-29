/***************************************************************************//**
 *   @file   adq2387.h
 *   @brief  Header file for adaq2387 Driver.
 *   @author Andrei Grozav (andrei.grozav@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <string.h>
#include "stdio.h"
#include "stdlib.h"
#include "adaq2387.h"
#include "delay.h"
#include "error.h"
#include "util.h"
#include "pwm.h"


/******************************************************************************/
/****************************** Macro definition ******************************/
/******************************************************************************/

#define NANO			0.000000001
#define MEGA			1000000
#define S_TWO_NANOS		1000000000

#define MIN_SAMPLING_FREQ	0.2*MEGA
#define MAX_SAMPLING_FREQ	15*MEGA

#define MIN_T_CNV		54*NANO
#define TYP_T_CNV		58*NANO
#define MAX_T_CNV		63*NANO

#define MIN_CYCLE_PERIOD	66.6*NANO
#define MAX_CYCLE_PERIOD	50000*NANO

#define T_CNV_H			5*NANO
#define T_CNV_L			8*NANO

// min
#define	T_FIRST_CLK		65*NANO
// max
#define	T_LAST_CLK		49*NANO

#define	MAX_CLK_HZ		400*MEGA

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/


/**
 * @brief Initialize GPIO driver handlers for the GPIOs in the system.
 *        adaq2387_init() helper function.
 * @param [out] dev - adaq2387_dev device handler.
 * @param [in] init_param - Pointer to the initialization structure.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t adaq2387_init_gpio(struct adaq2387_dev *dev,
				struct adaq2387_init_param *init_param)
{
	int32_t ret;

	ret = gpio_get_optional(&dev->gpio_two_lanes, init_param->gpio_two_lanes_init);
	if (ret != SUCCESS)
		return ret;

	ret = gpio_get_optional(&dev->gpio_test_pattern, init_param->gpio_test_pattern_init);
	if (ret != SUCCESS)
		return ret;

	/** Configure pins */
	if (init_param->gpio_two_lanes_init) {
		ret |= gpio_direction_output(dev->gpio_two_lanes, GPIO_LOW);
		ret |= gpio_direction_output(dev->gpio_test_pattern, init_param->two_lanes);
		if (ret != SUCCESS)
			return ret;

		mdelay(10);

		ret |= gpio_direction_output(dev->gpio_test_pattern, GPIO_LOW);
		if (ret != SUCCESS)
			return ret;

		mdelay(10);
	}

	return SUCCESS;
}

/**
 * @brief Read from device when converter has the channel sequencer activated.
 *        Enter register mode to read/write registers
 * @param [in] dev - adaq2387_dev device handler.
 * @param [out] buf - data buffer.
 * @param [in] samples - Number of samples per channel. For example, if  with
 * adaq2387_std_sequence_ch 2 channel where activated, buf will be filled with
 * 10 samples for each of them. If temp is enable, the there will be an other 10
 * samples for temperature
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t adaq2387_set_samplingrate(struct adaq2387_dev *dev,
			     uint32_t sampling_rate_sps)
{
	uint32_t ref_clk_hz;
	double   ref_period;
	double   sampling_period;

	uint32_t cnv_period_ns;
	uint32_t cnv_pulse_ns;
	uint32_t cnv_offset_ns;

	uint32_t clk_en_period_ns;
	uint32_t clk_en_pulse_ns;
	uint32_t clk_en_offset_ns;

	double   ref_clk_cycles_p_smpp;
	double   min_cnv_clk_periods;
	uint32_t n_acq_clk;
	double   acq_period;

	ref_clk_hz = dev->ref_clk_hz;
	ref_period = 1.0/ref_clk_hz;
	sampling_period = 1.0/sampling_rate_sps;

	ref_clk_cycles_p_smpp = ref_clk_hz/sampling_rate_sps;

	// number of aq clocks at DDR
	n_acq_clk = (dev->capture_data_width/N_LANES(dev->two_lanes))/ 2;
	acq_period = n_acq_clk * ref_period;

	min_cnv_clk_periods = DIV_ROUND_UP(5.0*NANO, (float)ref_period);
	if (min_cnv_clk_periods < 1)
		min_cnv_clk_periods = 1;

	////debug
	//printf("n_acq_clk %d\n", n_acq_clk);
	//printf("ref_clk_hz %d\n",ref_clk_hz);
	//printf("ref_period %.9lf\n",ref_period);
	//printf("sampling_period %.9lf\n",sampling_period);
	//printf("ref_clk_cycles_p_smpp %.9lf\n",ref_clk_cycles_p_smpp);
	//printf("acq_period %.9lf\n", acq_period);
	//printf("min_cnv_clk_periods %.9lf\n", min_cnv_clk_periods);

	// basic checks
	if (ref_clk_hz > 400*MEGA) {
		printf("Reference clock > 400 MHz\n");
		goto failure;
	}

	if (sampling_rate_sps > MAX_SAMPLING_FREQ) {
		printf("Sampling rate > max sampling freq\n");
		goto failure;
	}

	if (sampling_rate_sps < MIN_SAMPLING_FREQ) {
		printf("%d < %f",sampling_rate_sps, (float)MIN_SAMPLING_FREQ);
		printf("Sampling rate < min sampling freq\n");
		goto failure;
	}

	if (sampling_period < acq_period) {
		printf("Sampling period is less than min aquisition time\n");
		goto failure;
	}

	cnv_period_ns = sampling_period * S_TWO_NANOS;
	cnv_pulse_ns = min_cnv_clk_periods * ref_period * S_TWO_NANOS;
	cnv_offset_ns = 0;

        clk_en_period_ns = sampling_period * S_TWO_NANOS;
        clk_en_pulse_ns = n_acq_clk * ref_period * S_TWO_NANOS;
	clk_en_offset_ns = TYP_T_CNV * S_TWO_NANOS;

	//// debug
	printf("cnv_period %d\n",cnv_period_ns);
	printf("cnv_pulse  %d\n",cnv_pulse_ns );
	printf("cnv_offset %d\n",cnv_offset_ns);
	printf("clk_en_period %d\n",clk_en_period_ns);
	printf("clk_en_pulse  %d\n",clk_en_pulse_ns );
	printf("clk_en_offset %d\n",clk_en_offset_ns);

	pwm_set_period(dev->cnv_pwm_desc, cnv_period_ns);
	pwm_set_duty_cycle(dev->cnv_pwm_desc, cnv_pulse_ns);
	pwm_set_phase(dev->cnv_pwm_desc, cnv_offset_ns);

	pwm_set_period(dev->clk_en_pwm_desc, clk_en_period_ns);
	pwm_set_duty_cycle(dev->clk_en_pwm_desc, clk_en_pulse_ns);
	pwm_set_phase(dev->clk_en_pwm_desc, clk_en_offset_ns);

	pwm_enable(dev->clk_en_pwm_desc);

	return SUCCESS;

	failure:
		printf("%s FAILED.\n", __func__);
		return FAILURE;
}

/**
 * @brief Set test pattern.
 * @param [in] dev - adaq2387_dev device handler.
 * @param [in] test_pattern - active test_pattern.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t adaq2387_test_pattern(struct adaq2387_dev *dev,
				bool active)
{

	int32_t ret;
	ret = gpio_direction_output(dev->gpio_test_pattern, active);

	return ret;
}

/**
 * Initialize the device.
 * @param [out] device - The device structure.
 * @param [in] init_param - The structure that contains the device initial
 * 		parameters.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t adaq2387_init(struct adaq2387_dev **device,
		    struct adaq2387_init_param *init_param)
{
	struct adaq2387_dev *dev;
	int32_t ret;

	dev = (struct adaq2387_dev *)malloc(sizeof(*dev));
	if (!dev)
		goto error_dev;

	ret = adaq2387_init_gpio(dev, init_param);
	if (ret != SUCCESS)
		goto error_gpio;

	ret = pwm_init(&dev->cnv_pwm_desc, init_param->cnv_pwm_init);
	if (ret != SUCCESS)
		goto error_pwm;

	ret = pwm_init(&dev->clk_en_pwm_desc, init_param->clk_en_pwm_init);
	if (ret != SUCCESS)
		goto error_pwm;

	dev->sampling_rate_sps = init_param->sampling_rate_sps;
	dev->ref_clk_hz = init_param->ref_clk_hz;
	dev->capture_data_width = init_param->capture_data_width;
	dev->two_lanes = init_param->two_lanes;

	*device = dev;

	return ret;

error_gpio:
	gpio_remove(dev->gpio_test_pattern);
	gpio_remove(dev->gpio_two_lanes);
error_pwm:
	pwm_remove(dev->cnv_pwm_desc);
	pwm_remove(dev->clk_en_pwm_desc);
error_dev:
	free(dev);

	return FAILURE;
}

/**
 * @brief Free the memory allocated by adaq2387_init().
 * @param [in] dev - Pointer to the device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise
 */
int32_t adaq2387_remove(struct adaq2387_dev *dev)
{
	int32_t ret;

	if (!dev)
		return FAILURE;

	ret = pwm_remove(dev->cnv_pwm_desc);
	if (ret != SUCCESS)
		return ret;

	ret = pwm_remove(dev->clk_en_pwm_desc);
	if (ret != SUCCESS)
		return ret;

	ret = gpio_remove(dev->gpio_test_pattern);
	if (ret != SUCCESS)
		return ret;

	ret = gpio_remove(dev->gpio_test_pattern);
	if (ret != SUCCESS)
		return ret;

	free(dev);

	return ret;
}
