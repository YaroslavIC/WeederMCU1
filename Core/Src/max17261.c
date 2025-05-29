/*
 *  @file:  max17261.c
 *  @brief: MAX17261 Driver
 *
 *  Copyright (C) 2019 Libre Space Foundation (https://libre.space)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>

#include "max17261.h"

max17261_err_t
max17261_init( struct max17261_conf *conf)
{
	max17261_err_t ret;
	uint16_t value, model_cfg = 0x8000;

	// check for power on reset
	ret = max17261_read_word(conf, MAX17261_Status, &value);
	if (((value & 0x0002) || (conf->force_init)) && ret == 0) {
		// Delay until FSTAT.DNR bit == 0
		// TODO: Do we need a safety timeout here?
		do {
			ret = max17261_read_word(conf, MAX17261_FStat, &value);
		} while (ret || (value & 1));

		// Initialize Configuration
		ret = max17261_read_word(conf, MAX17261_HibCfg,
					 &conf->HibCFG); //Store original HibCFG value
		if (!ret)
			ret = max17261_write_word(conf, MAX17261_SoftWakeup,
						  0x90); // Exit Hibernate Mode step 1
		if (!ret)
			ret = max17261_write_word(conf, MAX17261_HibCfg,
						  0x0); // Exit Hibernate Mode step 2
		if (!ret)
			ret = max17261_write_word(conf, MAX17261_SoftWakeup,
						  0x0); // Exit Hibernate Mode step 3

		if (!ret)
			ret = max17261_write_word(conf, MAX17261_DesignCap, conf->DesignCap * 2);
		if (!ret)
			ret = max17261_write_word(conf, MAX17261_IChgTerm, conf->IchgTerm * 6.4);
		if (!ret)
			ret = max17261_write_word(conf, MAX17261_VEmpty, conf->VEmpty);

		if (ret)
			return ret;

		model_cfg |= (conf->ChargeVoltage > 4.275) << 10;
		model_cfg |= (conf->R100 & 1) << 13;

		// Write ModelCFG
		ret = max17261_write_word(conf, MAX17261_ModelCFG, model_cfg);
		if (ret)
			return ret;

		// Poll ModelCFG.Refresh(highest bit),
		// TODO: Do we need a safety timeout here?
		do {
			ret = max17261_read_word(conf, MAX17261_ModelCFG, &value);
			// do not continue until ModelCFG.Refresh==0
		} while (ret || (value & 0x8000));

		ret = max17261_write_word(conf, MAX17261_Config, 0xA210);
		// Init option 2
		if (conf->init_option == 2) {
			if (!ret)
				ret = max17261_write_verify(conf, MAX17261_RComp0,
							    conf->lparams.RCOMP0);
			if (!ret)
				ret = max17261_write_verify(conf, MAX17261_TempCo,
							    conf->lparams.TempCo);
			if (!ret)
				ret = max17261_write_verify(conf, MAX17261_QRTable00,
							    conf->lparams.QRTable[0]);
			if (!ret)
				ret = max17261_write_verify(conf, MAX17261_QRTable10,
							    conf->lparams.QRTable[1]);
			if (!ret)
				ret = max17261_write_verify(conf, MAX17261_QRTable20,
							    conf->lparams.QRTable[2]);
			if (!ret)
				ret = max17261_write_verify(conf, MAX17261_QRTable30,
							    conf->lparams.QRTable[3]);
		}

		// Restore Original HibCFG value
		if (!ret)
			ret = max17261_write_word(conf, MAX17261_HibCfg, conf->HibCFG);
		// Initialization complete
		if (!ret)
			ret = max17261_read_word(conf, MAX17261_Status, &value); // Read Status
		// Write and Verify Status with POR bit Cleared
		if (!ret)
			ret = max17261_write_verify(conf, MAX17261_Status, value
						    && 0xFFFD);
	}

	return ret;
}

uint8_t
max17261_get_SOC(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_read_word(conf, MAX17261_RepSOC, &value);
	return (uint8_t)(value >> 8);
}

max17261_err_t
max17261_set_reported_capacity(struct max17261_conf *conf, uint16_t capacity)
{
	return max17261_write_word(conf, MAX17261_RepCAP, capacity / CAPACITY_MULTIPLIER);
}

uint16_t
max17261_get_reported_capacity(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_read_word(conf, MAX17261_RepCAP, &value);
	return value * CAPACITY_MULTIPLIER;
}

max17261_err_t
max17261_set_full_reported_capacity(struct max17261_conf *conf,
                                    uint16_t capacity)
{
	return max17261_write_word(conf, MAX17261_FullRepCAP, capacity / CAPACITY_MULTIPLIER);
}

uint16_t
max17261_get_full_reported_capacity(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_read_word(conf, MAX17261_FullRepCAP, &value);
	return value * CAPACITY_MULTIPLIER;
}

max17261_err_t
max17261_set_design_capacity(struct max17261_conf *conf, uint16_t capacity)
{
	return max17261_write_word(conf, MAX17261_DesignCap, capacity / CAPACITY_MULTIPLIER);
}

uint16_t
max17261_get_design_capacity(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_read_word(conf, MAX17261_DesignCap, &value);
	return value * CAPACITY_MULTIPLIER;
}

uint16_t
max17261_get_voltage(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_read_word(conf, MAX17261_VCell, &value);
	//value *= VOLTAGE_MULTIPLIER_V;
	value = (uint16_t)(((float)value)* 1914.0 / 59071.0 );
	return value;
}

uint16_t
max17261_get_average_voltage(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_read_word(conf, MAX17261_AvgVCell, &value);
	value *= VOLTAGE_MULTIPLIER_V;
	return value;
}

max17261_err_t
max17261_reset_minmax_voltage(struct max17261_conf *conf)
{
	return max17261_write_word(conf, MAX17261_MaxMinVolt, 0x00FF);
}

max17261_err_t
max17261_get_minmax_voltage(struct max17261_conf *conf, uint16_t *min,
                            uint16_t *max)
{
	uint16_t value;
	max17261_err_t ret = max17261_read_word(conf, MAX17261_MaxMinVolt, &value);

	if (!ret) {
		*min = (value & 0xFF) * 20;
		*max = ((value >> 8) & 0xFF) * 20;
	}

	return ret;
}

int16_t
max17261_get_current(struct max17261_conf *conf)
{
	int16_t value;
	max17261_read_word(conf, MAX17261_Current, (uint16_t *) &value);
	value = value * CURRENT_MULTIPLIER;
	return value;
}

int16_t
max17261_get_average_current(struct max17261_conf *conf)
{
	int16_t value;
	max17261_read_word(conf, MAX17261_AvgCurrent, (uint16_t *) &value);
	value = value * CURRENT_MULTIPLIER;
	return value;
}

max17261_err_t
max17261_reset_minmax_current(struct max17261_conf *conf)
{
	return max17261_write_word(conf, MAX17261_MaxMinCurr, 0x00FF);
}

max17261_err_t
max17261_get_minmax_current(struct max17261_conf *conf, int16_t *min,
                            int16_t *max)
{
	uint16_t value;
	max17261_err_t ret = max17261_read_word(conf, MAX17261_MaxMinCurr, &value);

	if (!ret) {
		*min = ((int8_t)(value & 0xFF)) * CURRENT_MULTIPLIER_MINMAX;
		*max = ((int8_t)(value >> 8)) * CURRENT_MULTIPLIER_MINMAX;
	}

	return ret;
}

max17261_err_t
max17261_get_die_temperature(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_err_t ret = max17261_read_word(conf, MAX17261_DieTemp, &value);

	if (ret)
		return ret;

	return value >> 8;
}

max17261_err_t
max17261_get_temperature(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_err_t ret = max17261_read_word(conf, MAX17261_Temp, &value);

	if (ret)
		return ret;

	conf->temperature = value >> 8;
	return value >> 8;
}

max17261_err_t
max17261_get_average_temperature(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_err_t ret = max17261_read_word(conf, MAX17261_AvgTA, &value);

	if (ret)
		return ret;

	return value >> 8;
}

max17261_err_t
max17261_get_minmax_temperature(struct max17261_conf *conf, int8_t *min,
                                int8_t *max)
{
	uint16_t value;
	max17261_err_t ret = max17261_read_word(conf, MAX17261_MaxMinTemp, &value);
	if (!ret) {
		*min = value & 0xFF;
		*max = (value >> 8) & 0xFF;
	}

	return ret;
}

max17261_err_t
max17261_reset_minmax_temperature(struct max17261_conf *conf)
{
	return max17261_write_word(conf, MAX17261_MaxMinTemp,  0x807F);
}

uint16_t
max17261_get_TTE(struct max17261_conf *conf)
{
	uint16_t value;
	uint8_t ret = max17261_read_word(conf, MAX17261_TTE, &value);
	return ret ? 0 : value * TIME_MULTIPLIER_MIN;
}

max17261_err_t
max17261_get_learned_params(struct max17261_conf *conf)
{
	max17261_err_t ret;

	ret = max17261_read_word(conf, MAX17261_FullCapNom, &conf->lparams.FullCapNom);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_FullRepCAP, &conf->lparams.FullCapRep);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_RComp0, &conf->lparams.RCOMP0);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_TempCo, &conf->lparams.TempCo);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_Cycles, &conf->lparams.cycles);

	return ret;
}

max17261_err_t
max17261_restore_learned_params(struct max17261_conf *conf)
{
	max17261_err_t ret;

	ret = max17261_write_verify(conf, MAX17261_RComp0, conf->lparams.RCOMP0);
	if (!ret)
		ret = max17261_write_verify(conf, MAX17261_TempCo, conf->lparams.TempCo);
	if (!ret)
		ret = max17261_write_verify(conf, MAX17261_FullRepCAP,
					    conf->lparams.FullCapRep);
	if (!ret)
		ret = max17261_write_verify(conf, MAX17261_FullCapNom,
					    conf->lparams.FullCapNom);
	if (!ret)
		ret = max17261_write_verify(conf, MAX17261_dPAcc, 0x0C80);
	if (!ret)
		ret = max17261_write_verify(conf, MAX17261_dQAcc,
					    conf->lparams.FullCapNom * 2);
	if (!ret)
		ret = max17261_write_verify(conf, MAX17261_Cycles, conf->lparams.cycles);

	return ret;
}

uint8_t
max17261_get_qrtable_values(struct max17261_conf *conf)
{
	max17261_err_t ret;

	ret = max17261_read_word(conf, MAX17261_QRTable00, &conf->lparams.QRTable[0]);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_QRTable10, &conf->lparams.QRTable[1]);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_QRTable20, &conf->lparams.QRTable[2]);
	if (!ret)
		ret = max17261_read_word(conf, MAX17261_QRTable30, &conf->lparams.QRTable[3]);

	return ret;
}
/**
 * @brief I2C Read function wrapper
 * Reads 16bit data from device
 * @param conf
 * @param reg Register to read from
 * @param value Pointer to write value
 * @return 0 on success error code otherwise
 */
__attribute__((weak)) max17261_err_t
max17261_read_word(struct max17261_conf *conf, uint8_t reg, uint16_t *value)
{
	HAL_StatusTypeDef rt;
	uint8_t vl[2];
	uint16_t v;

	rt =  HAL_I2C_Mem_Read(&conf->hi2c , MAX17261_ADDRESS,  reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &vl, 2, 0x10000);
    v = (uint16_t)((((uint16_t)vl[0])<<8) | vl[1]);
    *value = v;

	return rt;
}

/**
 * @brief I2C Write function wrapper
 * Writes 16bit data to device
 * @param conf
 * @param reg Register to write to
 * @param value Value to write
 * @return 0 on success error code otherwise
 */
__attribute__((weak)) max17261_err_t
max17261_write_word(struct max17261_conf *conf, uint8_t reg, uint16_t value)
{
	uint8_t vl[2];
	vl[0] = (uint8_t)((value>>8) & 0xff);
	vl[1] = (uint8_t)(value & 0xff);

  return HAL_I2C_Mem_Write(&conf->hi2c , MAX17261_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &vl, 2, 0x10000);
}

/**
 * @brief I2C Write with verify function
 * Writes 16bit data to device and verifies content
 * @param conf
 * @param reg Register to write to
 * @param value Value to write
 * @return 0 on success error code otherwise
 */
max17261_err_t
max17261_write_verify(struct max17261_conf *conf, uint8_t reg, uint16_t value)
{
	uint8_t wcount = 0, ret;
	uint16_t readback;
	do {
		if ((ret = max17261_write_word(conf, reg, value)) != 0)
			return ret;

		max17261_delay_ms(conf, 1);

		if ((ret = max17261_read_word(conf, reg, &readback)) != 0)
			return ret;
	} while (readback != value && ++wcount < MAX17261_RB_THRESHOLD);
	return ((readback == value) ? 0 : 0x04);
}

/**
 * @brief Delay function
 * @param conf
 * @param period
 * @return
 */
__attribute__((weak)) max17261_err_t
max17261_delay_ms(struct max17261_conf *conf, uint32_t period)
{
  HAL_Delay(period);
  return 0;

}
