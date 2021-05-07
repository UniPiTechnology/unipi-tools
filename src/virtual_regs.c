/*
 * SPI communication with UniPi Neuron and Axon families of controllers
 *
 * Copyright (c) 2016  Faster CZ, ondra@faster.cz
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>

#include "armspi.h"
#include "armutil.h"
#include "virtual_regs.h"
#include "unipiutil.h"

typedef struct __attribute__ ((__packed__)) {
    uint16_t ao_sw;
    uint16_t ao_v_dev;
    uint16_t ao_v_offs;
    uint16_t ao_a_dev;
    uint16_t ao_a_offs;
    uint16_t ai_sw;
    uint16_t ai_v_dev1;
    uint16_t ai_v_offs1;
    uint16_t ai_a_dev1;
    uint16_t ai_a_offs1;
    uint16_t ai_v_dev2;
    uint16_t ai_v_offs2;
} Tcalibration;


int loaded = 0;
int use_calibration = 0;
Tcalibration calibration;
float vmul0, vmul1, vmul2, amul0, amul1;
float voffs0, voffs1, voffs2, aoffs0, aoffs1;

float fvalues[3]; // __attribute__((packed));

// function pointers
uint16_t (*ao_float2reg)(float);
float (*ao_reg2float)(uint16_t);
float (*ai1_reg2float)(uint16_t);
float (*ai2_reg2float)(uint16_t);


// without calibration
uint16_t aov_float2reg(float value)
{
    long lv = roundf(value/vmul0);
    if (lv < 0) return 0;
    if (lv > 4095) return 4095;
    return lv;
}

uint16_t aoa_float2reg(float value)
{
    long lv = roundf(value/amul0);
    if (lv < 0) return 0;
    if (lv > 4095) return 4095;
    return lv;
}

float aov_reg2float(uint16_t regvalue)
{
    return vmul0 * regvalue;
}

float aoa_reg2float(uint16_t regvalue)
{
    return amul0 * regvalue;
}

float aor_reg2float(uint16_t regvalue)
{
    return fvalues[2]; //return resistance - must be already calculated!!!
}

float aiv1_reg2float(uint16_t regvalue)
{
	if (((int16_t) regvalue) < 0) return 0.0;
    return vmul1 * regvalue;
}

float aia1_reg2float(uint16_t regvalue)
{
	if (((int16_t) regvalue) < 0) return 0.0;
    return amul1 * regvalue;
}

float aiv2_reg2float(uint16_t regvalue)
{
	if (((int16_t) regvalue) < 0) return 0.0;
    return vmul2 * regvalue;
}

float air2_reg2float(uint16_t regvalue)
{
    return  regvalue/10.0;
}

// WITH CALIBRATION
uint16_t aov_float2regc(float value)
{
    long lv = roundf((value-voffs0)/vmul0);
    if (lv < 0) return 0;
    if (lv > 4095) return 4095;
    return lv;
}

uint16_t aoa_float2regc(float value)
{
    long lv = roundf((value-aoffs0)/amul0);
    if (lv < 0) return 0;
    if (lv > 4095) return 4095;
    return lv;
}

float aov_reg2floatc(uint16_t regvalue)
{
    return vmul0 * regvalue + voffs0;
}

float aoa_reg2floatc(uint16_t regvalue)
{
    return amul0 * regvalue + aoffs0;
}

float aiv1_reg2floatc(uint16_t regvalue)
{
	if (((int16_t) regvalue) < 0) return 0.0;
    return vmul1 * regvalue + voffs1;
}

float aia1_reg2floatc(uint16_t regvalue)
{
	if (((int16_t) regvalue) < 0) return 0.0;
    return amul1 * regvalue + aoffs1;
}

float aiv2_reg2floatc(uint16_t regvalue)
{
	if (((int16_t) regvalue) < 0) return 0.0;
    return vmul2 * regvalue + voffs2;
}


void set_fp_by_mode(void)
{
    if (calibration.ao_sw == 3) {
        ao_float2reg = use_calibration ? aov_float2regc : aov_float2reg;
        ai2_reg2float = air2_reg2float;
        ao_reg2float = aor_reg2float;
    } else if (calibration.ao_sw == 1) {
        ao_float2reg = use_calibration ? aoa_float2regc : aoa_float2reg;
        ao_reg2float = use_calibration ? aoa_reg2floatc : aoa_reg2float;
        ai2_reg2float = use_calibration ? aiv2_reg2floatc : aiv2_reg2float;
    } else {
        ao_float2reg = use_calibration ? aov_float2regc : aov_float2reg;
        ao_reg2float = use_calibration ? aov_reg2floatc : aov_reg2float;
        ai2_reg2float = use_calibration ? aiv2_reg2floatc : aiv2_reg2float;
    }
    if (calibration.ai_sw == 1) {
        ai1_reg2float = use_calibration ? aia1_reg2floatc : aia1_reg2float;
    } else {
        ai1_reg2float = use_calibration ? aiv1_reg2floatc : aiv1_reg2float;
    }
}

void load_calibrating_const(arm_handle* arm)
{
    uint16_t vrefint, vref;
    int n;

    n = read_regs(arm, 1019, 12, (uint16_t*) &calibration);
    if (n != 12) return;
    n = read_regs(arm, 1009, 1, &vrefint);
    if (n != 1) return;
    n = read_regs(arm, 5, 1,  &vref);
    if (n != 1) return;
    vmul2 =  (3.3 * vrefint) / vref /4096;
    vmul0 = vmul1 = vmul2 * 3;
    amul0 = amul1 = vmul2 * 10;
    use_calibration = (calibration.ao_v_dev != 0xffff) && ((calibration.ao_v_dev!=0) || (calibration.ao_v_offs!=0));
    if (use_calibration) {
        voffs0 = vmul0*(calibration.ao_v_offs)/10000.0;
        vmul0  = vmul0*(1.0+calibration.ao_v_dev/10000.0);
        aoffs0 = amul0*(calibration.ao_a_offs)/10000.0;
        amul0  = amul0*(1.0+calibration.ao_a_dev/10000.0);
        voffs1 = vmul1*(calibration.ai_v_offs1)/10000.0;
        vmul1  = vmul1*(1.0+calibration.ai_v_dev1/10000.0);
        aoffs1 = amul1*(calibration.ai_a_offs1)/10000.0;
        amul1  = amul1*(1.0+calibration.ai_a_dev1/10000.0);
        voffs2 = vmul2*(calibration.ai_v_offs2)/10000.0;
        vmul2  = vmul2*(1.0+calibration.ai_v_dev2/10000.0);
    }
    set_fp_by_mode();
    loaded = 1;
    vvprintf("VIRTUAL REGS Vref=%5.3f use_calibration=%d\n", (3.3 * vrefint) / vref, use_calibration);
}


int read_virtual_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* result)
{
    int n, r0;
    uint16_t registers[3];

    if (arm && ((HW_BOARD(arm->bv.base_hw_version)==0)||(HW_BOARD(arm->bv.base_hw_version)==0xd))) { //
        if ((reg >= 3000) && (reg+cnt <= 3006)) { // Area of SW computed float values form Brain
            if (! loaded) {
                load_calibrating_const(arm);
                if (! loaded) return -1;
            }
            n = read_regs(arm, 2, 3, registers);         // ao, ai1, ai2
            if (n != 3) return -1;                       // illegal value
            fvalues[2] = ai2_reg2float(registers[2]);    // must be first
            fvalues[1] = ai1_reg2float(registers[1]);
            fvalues[0] = ao_reg2float(registers[0]);
            vvprintf("VIRTUAL REGS fvalues=%f %f %f\n",fvalues[0],fvalues[1],fvalues[2]);
            r0 = reg-3000;
            memcpy(result, ((uint16_t*)&fvalues)+r0, cnt*sizeof(uint16_t));
            return cnt;
        } else
            return 0;
    } else {
        return 0; // Illegal register address
    }
}

int prv_read_from_files(char** filelist, uint8_t filelist_size, uint16_t reg, uint8_t cnt, uint16_t* result)
{
	//if (reg + cnt > (sizeof(filelist) / sizeof(filelist[0])) || (reg < 0)) { //Check bounds
	if (reg + cnt > filelist_size || (reg < 0)) { //Check bounds
		return 0;
	}

	for (int i=reg; i < reg + cnt; i++) {
		vvprintf("Opening file %s \n", filelist[i]);
		FILE *fp;
		int value;
		fp = fopen(filelist[i] ,"r");
		if(fp == NULL){
			value = -1;
		}
		else{
			fscanf(fp, "%d", &value);
			fclose(fp);
		}

		memcpy(&((uint16_t*)result)[i-reg], &value, sizeof(uint16_t));
	}

    return cnt;
}

int read_pure_virtual_regs(uint16_t reg, uint8_t cnt, uint16_t* result)
{
	vvprintf("Reading pure virtual reg: %u cnt: %u\n", reg, cnt);

	reg = reg - OFFSET_PV_REGS;
        char ** filelist = NULL;
        uint8_t filelist_size = 0;
	// STORSTAT address range [OFFSET_PV_STORSTAT_GROUP : OFFSET_PV_LTE_GROUP)
	if (reg < OFFSET_PV_LTE_GROUP){
		char* filelist_a[] = {"/var/run/unipi_stats/cycles_used",
				"/var/run/unipi_stats/good_blocks",
				"/var/run/unipi_stats/power_cycles",
				"/var/run/unipi_stats/vendor_1",
				"/var/run/unipi_stats/vendor_2",
				"/var/run/unipi_stats/vendor_3"};
		filelist = filelist_a;
                filelist_size = (sizeof(filelist_a) / sizeof(filelist_a[0]));
	}
	// LTE address range [OFFSET_PV_LTE_GROUP : OFFSET_PV_SYSSTAT_GROUP)
	else if (reg < OFFSET_PV_SYSSTAT_GROUP){
		char* filelist_b[] = {"/var/run/unipi_lte/rssi",
				"/var/run/unipi_lte/mode",
				"/var/run/unipi_lte/nettype",
				"/var/run/unipi_lte/sigqual"};
		filelist = filelist_b;
                filelist_size = (sizeof(filelist_b) / sizeof(filelist_b[0]));

	}
	else{
		return 0;
        }


	return prv_read_from_files(filelist, filelist_size, reg, cnt, result);

}

int write_virtual_regs(arm_handle* arm, uint16_t reg, uint8_t cnt, uint16_t* values)
{
    float fval;
    uint32_t swapped;
    uint16_t regval;
    if (arm && ((HW_BOARD(arm->bv.base_hw_version)==0)||(HW_BOARD(arm->bv.base_hw_version)==0xd))) {
        if ((reg >= 3000) && (reg+cnt <= 3006)) {
            if (! loaded) {
                load_calibrating_const(arm);
                if (! loaded) return -1;
            }
            if ((reg==3000) && (cnt >=2)) {
                fval = *((float*)values);
                vvprintf("VIRTUAL REGS write fval=%f\n",fval);
                regval = ao_float2reg(fval);
                if (write_regs(arm, 2, 1, &regval)!=1) return -1;
                return cnt;
            } else {
                return -1;
            }
        } else if ((reg == 3006) && (cnt==2)) {
            if (! loaded) {
                load_calibrating_const(arm);
                if (! loaded) return -1;
            }
            swapped = (uint32_t)(*(values+1) | (*(values) << 16));
            fval = *((float*) &swapped);
            vvprintf("VIRTUAL REGS write fval=%f\n",fval);
            regval = ao_float2reg(fval);
            if (write_regs(arm, 2, 1, &regval)!=1) return -1;
            return cnt;
        }
    }
    return 0;
}

void monitor_virtual_regs(arm_handle* arm, uint16_t reg, uint16_t* result)
{
    if ((arm == NULL) || ((HW_BOARD(arm->bv.base_hw_version)!=0)&&(HW_BOARD(arm->bv.base_hw_version)!=0xd)))
        return;
    // do only for Brain
    if (reg == 1019) {
        if (! loaded) {
            load_calibrating_const(arm);
            if (! loaded) return;
        }
        calibration.ao_sw = *result;
        set_fp_by_mode();
        vvprintf("VIRTUAL REGS ao mode=%d\n",calibration.ao_sw);
    } else if (reg == 1024) {
        if (! loaded) {
            load_calibrating_const(arm);
            if (! loaded) return;
        }
        calibration.ai_sw = *result;
        set_fp_by_mode();
        vvprintf("VIRTUAL REGS ai mode=%d\n",calibration.ai_sw);
    }
}

void monitor_virtual_coils(arm_handle* arm, uint16_t reg, uint8_t* values, uint16_t cnt, int platform)
{

    char* gpio_1w_reset_path = NULL;

    if (platform == VIRTUAL_COILS_NANOPI)
        gpio_1w_reset_path = "/sys/class/gpio/gpio18/value";
    else if(platform == VIRTUAL_COILS_ZULU)
        gpio_1w_reset_path = "/sys/class/gpio/gpio149/value";

    int gpio = open(gpio_1w_reset_path, O_WRONLY);
    if (gpio < 0) return;

	int shift = 1001 - reg;
    if (((values[0] >> shift) & 1)== 0) {
        vvprintf("VIRTUAL COIL 1001 mode=on d=%02x\n", values[0]);
		write(gpio,"1", 1);
	} else {
        vvprintf("VIRTUAL COIL 1001 mode=off d=%02x\n", values[0]);
		write(gpio,"0", 1);
	}
    close(gpio);
}
//int read_virtual_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* result);
//int write_virtual_bit(arm_handle* arm, uint16_t reg, uint8_t value, uint8_t do_lock);
//int write_virtual_bits(arm_handle* arm, uint16_t reg, uint16_t cnt, uint8_t* values);


