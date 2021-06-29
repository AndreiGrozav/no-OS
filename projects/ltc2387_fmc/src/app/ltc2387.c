/***************************************************************************//**
 *   @file   adaq2387.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *   @author AntoniuMiclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "parameters.h"
#include <xparameters.h>
#include <xil_printf.h>
#include <xil_cache.h>
#include "gpio.h"
#include "gpio_extra.h"
#include "delay.h"
#include "error.h"
#include "axi_adc_core.h"
#include "axi_dmac.h"
#include "adaq2387.h"
#include "pwm.h"
#include "axi_pwm_extra.h"
#include "clk_axi_clkgen.h"
#ifdef IIO_SUPPORT
#include "app_iio.h"
#endif

int main(void)
{
	printf("sarakie de cod\n");
	int32_t status;


	struct xil_gpio_init_param xil_gpio_param = {
#ifdef PLATFORM_MB
		.type = GPIO_PL,
#else
		.type = GPIO_PS,
#endif
		.device_id = GPIO_DEVICE_ID
	};

	struct gpio_init_param gpio_test_pat_param = {
		.number = GPIO_TEST_PATTERN
	};

	gpio_test_pat_param.platform_ops = &xil_gpio_platform_ops;
	gpio_test_pat_param.extra = &xil_gpio_param;

	// gpio_desc *gpio_test_pattern;


	struct gpio_init_param gpio_two_lanes_param = {
		.number = GPIO_TWO_LANES
	};

	gpio_two_lanes_param.platform_ops = &xil_gpio_platform_ops;
	gpio_two_lanes_param.extra = &xil_gpio_param;

	// gpio_desc *gpio_two_lanes;

	uint32_t ref_clk_rate = 50000000;

	struct axi_adc_init adaq2387_core_param = {
		.name = "adaq2387_adc",
		.base = AXI_CORE_BASEADDR,
		.num_channels = 1
	};
	struct axi_adc	*adaq2387_core;

	struct axi_dmac_init adaq2387_dmac_param = {
		.name = "adaq2387_dmac",
		.base = AXI_DMA_BASEADDR,
		.direction = DMA_DEV_TO_MEM,
		.flags = 0
	};
	struct axi_dmac *adaq2387_dmac;

        struct axi_clkgen_init clkgen_delay_init = {
                .name = "clkgen_delay",
                .base = AXI_CLKGEN_DELAY_BASEADDR,
                .parent_rate = 100000000,
        };

        struct axi_clkgen_init clkgen_init = {
                .name = "clkgen",
                .base = AXI_CLKGEN_BASEADDR,
                .parent_rate = 100000000,
        };

	struct axi_pwm_init_param axi_pwm_init_0 = {
		.base_addr = AXI_PWMGEN_BASEADDR,
		.ref_clock_Hz = ref_clk_rate,
		.channel = 0,
	};

	struct pwm_init_param cnv_pwm_init = {
		.period_ns = 100,		/* 100Khz */
		.duty_cycle_ns = 40,
		.polarity = PWM_POLARITY_HIGH,
		.extra = &axi_pwm_init_0,
	};

	struct axi_pwm_init_param axi_pwm_init_1 = {
		.base_addr = AXI_PWMGEN_BASEADDR,
		.ref_clock_Hz = ref_clk_rate,
		.channel = 1,
	};

	struct pwm_init_param clk_en_pwm_init = {
		.period_ns = 100,		/* 100Khz */
		.duty_cycle_ns = 20,
		.polarity = PWM_POLARITY_HIGH,
		.extra = &axi_pwm_init_1,
	};

	struct axi_clkgen *axi_clkgen_delay;
	struct axi_clkgen *axi_clkgen;

	struct adaq2387_init_param adaq2387_init_param = {
		.cnv_pwm_init = &cnv_pwm_init,
		.clk_en_pwm_init = &clk_en_pwm_init,
		.gpio_two_lanes_init = &gpio_two_lanes_param,
		.gpio_test_pattern_init = &gpio_test_pat_param,
		.sampling_rate_sps = 1000000,
		.ref_clk_hz = ref_clk_rate,
		.capture_data_width = 16,
		.two_lanes = true,
	};

	struct adaq2387_dev *dev;

	int ret = 0;

	ret = axi_clkgen_init(&axi_clkgen_delay, &clkgen_delay_init);
	if (ret < 0)
		return ret;

	axi_clkgen_set_rate(axi_clkgen_delay, 200000000);

	ret = axi_clkgen_init(&axi_clkgen, &clkgen_init);
	if (ret < 0)
		return ret;

	ret = axi_clkgen_set_rate(axi_clkgen, ref_clk_rate);
	if (ret < 0)
		return ret;

	ret = adaq2387_init(&dev, &adaq2387_init_param);
	if (ret < 0)
		return ret;

	// interface core setup
	printf("adc init\n");
	status = axi_adc_init(&adaq2387_core,  &adaq2387_core_param);
	if (status != SUCCESS) {
		printf("axi_adc_init() error: %s\n", adaq2387_core->name);
	}

	//// get rate se BLOCHEAZA
	//uint32_t rate;
	//axi_clkgen_get_rate(&axi_clkgen, &rate);
	//printf("clk_gen = %x\n", rate);

	// PRBS test
	adaq2387_test_pattern(dev, adaq2387_TEST_ON);

	adaq2387_set_samplingrate(dev, 1000000);

	//// test reliability at multiple configurations

	// int i, j;
	// for (i = 30; i <= 200; i=i+10) {
	// 	ret = axi_clkgen_set_rate(axi_clkgen, i*1000000);

	// 	if (ret < 0) {
	// 		printf("can't set clk freq %d\n",i);
	// 		continue;
	// 	}

	// 	for (j = 5; j <= 15; j=j+1) {
	// 		ret = adaq2387_set_samplingrate(dev, j*1000000);
	// 		printf("sampling rate %d M - ", j);
	// 		if (ret < 0) {
	// 			printf("can't set rate %d\n",j);
	// 			continue;
	// 		}
	// 		ret = axi_adc_delay_calibrate(adaq2387_core, 2, AXI_ADC_PN_CUSTOM);
	// 		uint32_t reg_cntrl;
	// 		if (ret != 0) {
	// 			axi_adc_read(adaq2387_core, AXI_ADC_REG_CNTRL, &reg_cntrl);
	// 			reg_cntrl |= AXI_ADC_DDR_EDGESEL;
	// 			axi_adc_write(adaq2387_core, AXI_ADC_REG_CNTRL, reg_cntrl);
	// 			axi_adc_delay_calibrate(adaq2387_core, 2, AXI_ADC_PN_CUSTOM);
	// 		}
	// 	}
	// 	printf ("\n --------------------------------------------- \n");
	// }

	if (axi_adc_pn_mon(adaq2387_core, AXI_ADC_PN_CUSTOM, 10) == -1) {
		printf("%s Test pattern mismatch!\n", __func__);
	};

	// capture data with DMA
	//adaq2387_test_pattern(dev, adaq2387_TEST_OFF); // gpio

	axi_dmac_init(&adaq2387_dmac, &adaq2387_dmac_param);

	axi_dmac_transfer(adaq2387_dmac, ADC_DDR_BASEADDR,
			  16384 * 2);
	// printf("Data captured. Start address 0x%x\n", );

#ifdef IIO_SUPPORT
	printf("The board accepts libiio clients connections through the serial backend.\n");

	struct iio_axi_adc_init_param iio_axi_adc_init_par;
	iio_axi_adc_init_par = (struct iio_axi_adc_init_param) {
		.rx_adc = adaq2387_core,
		.rx_dmac = adaq2387_dmac,
#ifndef PLATFORM_MB
		.dcache_invalidate_range = (void (*)(uint32_t,
						     uint32_t))Xil_DCacheInvalidateRange
#endif
	};

	return iio_server_init(&iio_axi_adc_init_par);
#endif

	printf("adaq2387: setup and configuration is done\n");

	return(0);
}
