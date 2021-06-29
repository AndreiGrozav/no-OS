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

#ifndef SRC_adaq2387_H_
#define SRC_adaq2387_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "pwm.h"
#include "gpio.h"

/******************************************************************************/
/******************************* Definitions **********************************/
/******************************************************************************/

#define adaq2387_TEST_OFF	false
#define adaq2387_TEST_ON	true
#define N_LANES(x)		((x == true) ? 2 : 1)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @struct adaq2387_init_param
 * @brief  Structure containing the init parameters needed by the adaq2387 device
 */
struct adaq2387_init_param {
	/* PWM generator init structure */
	struct pwm_init_param	*cnv_pwm_init;
	struct pwm_init_param	*clk_en_pwm_init;
	/** Number of lanes GPIO initialization structure. */
	struct gpio_init_param	*gpio_two_lanes_init;
	/** Test pattern GPIO initialization structure. */
	struct gpio_init_param	*gpio_test_pattern_init;
	/* Sampling rate */
	uint32_t		sampling_rate_sps;
	/* Reference clk speed */
	uint32_t		ref_clk_hz;
	/* Capture data width */
	uint8_t			capture_data_width;
	/* Two lanes */
	bool			two_lanes;
};

/**
 * @struct adaq2387_dev
 * @brief  Structure representing an adaq2387 device
 */
struct adaq2387_dev {
	/* Conversion PWM generator descriptor */
	struct pwm_desc		*cnv_pwm_desc;
	/* CLK gate PWM generator descriptor */
	struct pwm_desc		*clk_en_pwm_desc;
	/** Number of lanes GPIO handler. */
	struct gpio_desc	*gpio_two_lanes;
	/** Test pattern GPIO handler. */
	struct gpio_desc	*gpio_test_pattern;
	/* Sampling rate */
	uint32_t		sampling_rate_sps;
	/* Reference clk speed */
	uint32_t		ref_clk_hz;
	/* Capture data width */
	uint8_t			capture_data_width;
	/* Two lanes */
	bool			two_lanes;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Configure test mode */
int32_t adaq2387_test_pattern(struct adaq2387_dev *dev,
			       bool test_pattern);

/* Configure sampling rate */
int32_t adaq2387_set_samplingrate(struct adaq2387_dev *dev,
			       uint32_t sampling_rate_sps);

/* Initialize the device. */
int32_t adaq2387_init(struct adaq2387_dev **device,
		    struct adaq2387_init_param *init_param);

/* Remove the device and release resources. */
int32_t adaq2387_remove(struct adaq2387_dev *dev);

#endif /* SRC_adaq2387_H_ */
