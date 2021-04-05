/*
 * Copyright (C) 2015 Formosa Measurement Technology Inc. Ltd. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

/* Driver description
 *
 * File Name    :
 * Authors      : conrad@fmti.com.tw
 * Version      : 1.0.0
 * Date         : 2018/4/23
 * Description  : FBM320 pressure sensor API for MCU of ARM_M0 core
 *
 */

/* Revised history
 * 1.0.0: first release
 *
 */

#define SPI
#define SPI_4_WIRES_MODE // CS available
#define FBM_320 // FBM_320 FBM_325 FBM_GIT

#include "fbm.h"
#include "spi.h"

#include <string.h>

//volatile uint32_t TMR0_Ticks; // replaced by HAL_GetTick()
volatile uint32_t fbm320_update_rdy;

static void fbm320_us_delay(uint32_t us);
#ifdef SPI
static uint8_t fbm320_spi_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
static uint8_t fbm320_spi_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
#else
static uint8_t fbm320_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
static uint8_t fbm320_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
#endif
static int32_t fbm320_startMeasure_temp(struct fbm320_data *barom);
static int32_t fbm320_get_raw_temperature(struct fbm320_data *barom);
static int32_t fbm320_startMeasure_press(struct fbm320_data *barom);
static int32_t fbm320_get_raw_pressure(struct fbm320_data *barom);
static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom);
static int32_t fbm320_set_oversampling_rate(struct fbm320_data *barom , enum fbm320_osr osr_setting);
static int32_t fbm320_chipid_check(struct fbm320_data *barom);
static int32_t fbm320_version_identification(struct fbm320_data *barom);
static int32_t fbm320_calculation(struct fbm320_data *barom);
//static int32_t fbm325_calculation(struct fbm320_data *barom);
static int32_t pressure_altitude_conversion(int32_t real_pressure);

/**
 * { pointer of fbm320 data }
 */
static struct fbm320_data fbm320_barom;
struct fbm320_data *barom = &fbm320_barom;

#ifdef SPI
static uint8_t fbm320_spi_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
{
	HAL_StatusTypeDef _result = HAL_OK;

	uint8_t _data_tx[cnt + 2];
	uint8_t _cmd = 0;

	memset(&_data_tx, 0, cnt+2);

	switch (cnt) {
		case 1:
			_cmd = FBM320_SPI_WRITE | FBM320_SPI_1BYTE;
			break;
		case 2:
			_cmd = FBM320_SPI_WRITE | FBM320_SPI_2BYTE;
			break;
		case 3:
			_cmd = FBM320_SPI_WRITE | FBM320_SPI_3BYTE;
			break;
		default:
			_cmd = FBM320_SPI_WRITE | FBM320_SPI_4BYTE;
	}

	_data_tx[0] = _cmd;
	_data_tx[1] = reg_addr + cnt - 1;
	memcpy(&_data_tx[2], reg_data, cnt);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	_result = HAL_SPI_Transmit(&hspi2, _data_tx, cnt + 2, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	if (_result != HAL_OK) {
		return -1;
	}

	return 0;

//	/* This is just an example. This function have to
//	   be implemented according to your platform. */
//	uint8_t cmd, i;
//
//	switch (cnt) {
//	case 1:
//		cmd = FBM320_SPI_WRITE | FBM320_SPI_1BYTE;
//		break;
//	case 2:
//		cmd = FBM320_SPI_WRITE | FBM320_SPI_2BYTE;
//		break;
//	case 3:
//		cmd = FBM320_SPI_WRITE | FBM320_SPI_3BYTE;
//		break;
//	default:
//		cmd = FBM320_SPI_WRITE | FBM320_SPI_4BYTE;
//	}
//	SPI_SET_SS0_LOW(SPI0);
//	/* Write to TX register */
//	SPI_WRITE_TX0(SPI0, cmd);
//	/* Trigger SPI data transfer */
//	SPI_TRIGGER(SPI0);
//	while (SPI_IS_BUSY(SPI0));
//	SPI_WRITE_TX0(SPI0, (reg_addr + (cnt - 1)));
//	SPI_TRIGGER(SPI0);
//	while (SPI_IS_BUSY(SPI0));
//	for (i = 0; i < cnt; i++) {
//		SPI_WRITE_TX0(SPI0, *(reg_data + i));
//		SPI_TRIGGER(SPI0);
//		/* Check SPI0 busy status */
//		while (SPI_IS_BUSY(SPI0));
//	}
//	SPI_SET_SS0_HIGH(SPI0);
//
//	return 0;
}



static uint8_t fbm320_spi_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
{
	HAL_StatusTypeDef _result = HAL_OK;

	uint8_t _cmd = 0;
	uint8_t _data_tx[2];
	uint8_t _data_rx[cnt];

	switch (cnt) {
	case 1:
		_cmd = FBM320_SPI_READ | FBM320_SPI_1BYTE;
		break;
	case 2:
		_cmd = FBM320_SPI_READ | FBM320_SPI_2BYTE;
		break;
	case 3:
		_cmd = FBM320_SPI_READ | FBM320_SPI_3BYTE;
		break;
	default:
		_cmd = FBM320_SPI_READ | FBM320_SPI_4BYTE;
	}

	_data_tx[0] = _cmd;
	_data_tx[1] = reg_addr + (cnt - 1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	_result = HAL_SPI_Transmit(&hspi2, _data_tx, 2, 100);
	if (_result != HAL_OK) {
		return -1;
	}

	_result = HAL_SPI_Receive(&hspi2, _data_rx, cnt, 100);
	if (_result != HAL_OK) {
		return -1;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	memcpy(reg_data, _data_rx, cnt);

	return 0;

//	/* This is just an example. This function have to
//	   be implemented according to your platform. */
//
//	int8_t i;
//	uint8_t cmd;
//	uint32_t tmp;
//
//	switch (cnt) {
//	case 1:
//		cmd = FBM320_SPI_READ | FBM320_SPI_1BYTE;
//		break;
//	case 2:
//		cmd = FBM320_SPI_READ | FBM320_SPI_2BYTE;
//		break;
//	case 3:
//		cmd = FBM320_SPI_READ | FBM320_SPI_3BYTE;
//		break;
//	default:
//		cmd = FBM320_SPI_READ | FBM320_SPI_4BYTE;
//	}
//	SPI_SET_SS0_LOW(SPI0);
//	/* Write to TX register */
//	SPI_WRITE_TX0(SPI0, cmd);
//	SPI_TRIGGER(SPI0);
//	while (SPI_IS_BUSY(SPI0));
//	SPI_WRITE_TX0(SPI0, reg_addr + (cnt - 1));
//	SPI_TRIGGER(SPI0);
//	while (SPI_IS_BUSY(SPI0));
//	for (i = (cnt - 1); i >= 0; i--) {
//		SPI_WRITE_TX0(SPI0, 0x00);//dummy clock
//		SPI_TRIGGER(SPI0);
//		while (SPI_IS_BUSY(SPI0));
//		tmp = SPI_READ_RX0(SPI0);
////			printf("SPI read: %#x\n\r", tmp);
//		*(reg_data + i) = tmp;
//	}
//	SPI_SET_SS0_HIGH(SPI0);
//
//	return 0;
}
#endif



// i2c not implemented due to missing test caps
//#ifdef I2C
//static uint8_t fbm320_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
//{
//	/* This is just an example. This function have to
//	   be implemented according to your platform. */
//	uint8_t status;
//	uint32_t cnt_write;
//	cnt_write = I2C_WriteMultiBytesOneReg(I2C0, FBM320_I2C_SLAVE_ADDR, reg_addr \
//	                                      , reg_data, cnt);
//	status = (cnt_write > 0) ?  0 : 1;
//	return status;
//}
//static uint8_t fbm320_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
//{
//	/* This is just an example. This function have to
//	   be implemented according to your platform. */
//	uint8_t status;
//	uint32_t cnt_read;
//
//	cnt_read = I2C_ReadMultiBytesOneReg(I2C0, FBM320_I2C_SLAVE_ADDR\
//	                                    , reg_addr, reg_data, cnt);
//	status = (cnt_read > 0) ?  0 : 1;
//	return status;
//}
//#endif




/**
 * @brief      { API for fbm320 delay }
 *
 * @param[in]  us    { delay time in microseconds }
 */
static void fbm320_us_delay(uint32_t us)
{
	HAL_Delay(us / 1000);
//	/* This is just an example. This function have to
//	   be implemented according to your platform. */
//	CLK_SysTickDelay(us);
}



/**
 * @brief      { API for assigning function pointers, as bus read/write
 *               and delay. }
 *
 * @return     { 0, succeeded
 *              -1, failed }
 */
int8_t fbm320_init(void)
{
	int32_t err;
	uint8_t data_buf;

	fbm320_barom.delay_usec = fbm320_us_delay;
	/* The minimum start up time of fbm320 is 15ms */
	barom->delay_usec(1000 * 15);

#ifdef SPI
	fbm320_barom.bus_write = fbm320_spi_writeblock;
	fbm320_barom.bus_read = fbm320_spi_readblock;
	/* The default of SPI is in 3 wires mode after power on reset. If 4 wires SPI
    mode is preffered, the following statements will be needed. */
	#ifdef SPI_4_WIRES_MODE
		/* Set SPI bus as 4 wires mode */
		data_buf = FBM320_SPI_CTRL_REG_SDO_ACTIVE_EN;
		barom->bus_write(FBM320_SPI_CTRL_REG, sizeof(uint8_t), &data_buf);
	#endif
#else
	fbm320_barom.bus_write = fbm320_i2c_writeblock;
	fbm320_barom.bus_read = fbm320_i2c_readblock;
#endif

	err = fbm320_chipid_check(barom);
	if (err) {
		err = -1;
		goto err_chip_id_chk;
	} else {
#ifdef DEBUG_FBM320
		printf("%s:fbm320_chipid_check() passed!\n", __func__);
#endif
	}

	err = fbm320_version_identification(barom);
	if (err) {
		err = -2;
		goto err_version_identification;
	} else {
#ifdef DEBUG_FBM320
		printf("%s:fbm320_version_identification() passed!\n", __func__);
#endif
	}

	err = fbm320_read_store_otp_data(barom);
	if (err) {
		err = -3;
		goto err_read_otp;
	} else {
#ifdef DEBUG_FBM320
		printf("%s:fbm320_read_store_otp_data() passed!\n", __func__);
#endif//DEBUG_FBM330
	}
	err = 0;

	fbm320_set_oversampling_rate(barom, OVERSAMPLING_RATE_DEFAULT);
	/* Setting the P_CONFIG_REG_GAIN */
#define P_CONFIG_REG_GAIN_SETTING FBM320_P_CONFIG_REG_GAIN_X32

	barom->bus_read(FBM320_P_CONFIG_REG, sizeof(uint8_t), &data_buf);

	data_buf &= ~(FBM320_P_CONFIG_REG_GAIN_MAK);
	data_buf |= P_CONFIG_REG_GAIN_SETTING;

	barom->bus_write(FBM320_P_CONFIG_REG, sizeof(uint8_t), &data_buf);

#ifdef DEBUG_FBM320
	printf("%s:Setting of FBM320_P_CONFIG_REG_GAIN: %#x\n", __func__, P_CONFIG_REG_GAIN_SETTING);
#endif

#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() succeeded!\n", __func__);
#endif
	return err;

err_chip_id_chk:
#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() failed!; fbm320_ID:%#x,err:%d\n", __func__, fbm320_barom.chip_id, err);
#endif
	return err;
err_version_identification:
#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() failed!; fbm320 version:%#x,err:%d\n", __func__, fbm320_barom.hw_ver, err);
#endif
	return err;
err_read_otp:
#ifdef DEBUG_FBM320
	printf("%s:fbm320_init() failed!; fbm320 otp reading failed!,err:%d\n", __func__, err);
#endif
	return err;
}



int32_t fbm320_read_raw_t(void)
{
	return barom->raw_temperature;
}
/**
 * @brief      API for read real temperature value in unit of degree Celsius
 *
 * @return     { temperature value in unit of degree Celsius }
 */



float fbm320_read_temperature(void)
{
	fbm320_calculation(barom);
	return barom->real_temperature * 0.01;
}



int32_t fbm320_read_raw_p(void)
{
	return barom->raw_pressure;
}



/**
 * @brief      API for read real pressure value in unit of Pa
 *
 * @return     { pressure value in unit of Pa }
 */
float fbm320_read_pressure(void)
{
	fbm320_calculation(barom);
	return barom->real_pressure * 0.125;
}



/**
 * @brief      API for read real temperature and pressure values
 *             stored in fbm320_data structure
 *
 * @param      real_pressure     The pointer for saving real pressure value
 *                               Pressure unit: 0.125 Pa
 * @param      real_temperature  The pointer for saving real temperature value
 *                               Temperature unit: 0.01 degree Celsius
 */
void fbm320_read_data(int32_t *real_pressure, int32_t *real_temperature)
{
	fbm320_calculation(barom);
	*real_pressure = barom->real_pressure;
	*real_temperature = barom->real_temperature;
	return;
}



/**
 * @brief      { This api ignite a measurement procedure. It writes data into
 *               the register of FBM320_TAKE_MEAS_REG. }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_write() }
 */
static int32_t fbm320_startMeasure_temp(struct fbm320_data *barom)
{
	int err;
	uint8_t bus_wr_data;

	bus_wr_data = FBM320_MEAS_TEMP;
	err = barom->bus_write(FBM320_TAKE_MEAS_REG, sizeof(uint8_t), &bus_wr_data);

	return err;
}



/**
 * @brief      { This api gets the data from the registers of FBM320_READ_MEAS_REG_U
 *               , FBM320_READ_MEAS_REG_L and FBM320_READ_MEAS_REG_XL. And the data are
 *               stored in "barom->raw_temperature". }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_read() }
 */
static int32_t fbm320_get_raw_temperature(struct fbm320_data *barom)
{
	int err;
	uint8_t buf[3] = {0};

	err = barom->bus_read(FBM320_READ_MEAS_REG_U, 3 * sizeof(uint8_t), buf);
//	barom->raw_temperature = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];
	barom->raw_temperature = ((uint32_t)buf[2] << 16) + ((uint32_t)buf[1] << 8) + buf[0];

#ifdef DEBUG_FBM320
	printf("%s: uncompensated temperature: %d\n", DEVICE_NAME, barom->raw_temperature);
#endif
	return err;
}
/**
 * @brief      { This api ignite a measurement procedure. It writes data into
 *               the register of FBM320_TAKE_MEAS_REG. }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_write() }
 */
static int32_t fbm320_startMeasure_press(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t bus_wr_data;

	bus_wr_data = barom->cmd_start_p;
	err = barom->bus_write(FBM320_TAKE_MEAS_REG, sizeof(uint8_t), &bus_wr_data);

	return err;
}
/**
 * @brief      { This api gets the data from the registers of FBM320_READ_MEAS_REG_U
 *               , FBM320_READ_MEAS_REG_L and FBM320_READ_MEAS_REG_XL. And the data are
 *               stored in "barom->raw_temperature". }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { return of bus_read() }
 */
static int32_t fbm320_get_raw_pressure(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t buf[3] = {0};

	err = barom->bus_read(FBM320_READ_MEAS_REG_U, 3 * sizeof(uint8_t), buf);
//	barom->raw_pressure = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];
	barom->raw_pressure = ((uint32_t)buf[2] << 16) + ((uint32_t)buf[1] << 8) + buf[0];

#ifdef DEBUG_FBM320
	printf("%s: uncompensated pressure:  %d\n", DEVICE_NAME, barom->raw_pressure);
#endif

	return err;
}



/**
 * @brief      { API for reading hardware version }
 *
 * @param      barom  FBM320 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int32_t fbm320_version_identification(struct fbm320_data *barom)
{
	int err;
	uint8_t buf[2] = {0};
	uint8_t version = 0;
	uint8_t bus_wr_data;

	bus_wr_data = FBM320_SOFTRESET_CMD;
	barom->bus_write(FBM320_SOFTRESET_REG, sizeof(uint8_t), &bus_wr_data);

	barom->delay_usec(1000 * 15); /* The minimum start up time of fbm320 is 15ms */

	err = barom->bus_read(FBM320_TAKE_MEAS_REG, sizeof(uint8_t), buf);

	err = barom->bus_read(FBM320_VERSION_REG, sizeof(uint8_t), buf + 1);

	version = ((buf[0] & 0xC0) >> 6) | ((buf[1] & 0x70) >> 2);
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
	printf("%s: The value of version: %#x\n", __func__, version);
#endif

	switch (version)	{
	case hw_ver_b1:
		barom->hw_ver = hw_ver_b1;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is B1.\n", __func__);
#endif
		break;
	case hw_ver_b2:
		barom->hw_ver = hw_ver_b2;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is B2.\n", __func__);
#endif
		break;
	case hw_ver_b3:
		barom->hw_ver = hw_ver_b3;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is B3.\n", __func__);
#endif
		break;
	case hw_ver_b4:
		barom->hw_ver = hw_ver_b4;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is B4.\n", __func__);
#endif
		break;
    case hw_ver_b5:
		barom->hw_ver = hw_ver_b5;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is B5.\n", __func__);
#endif//DEBUG_FBM320
		break;
	default:
		barom->hw_ver = hw_ver_unknown;
#if defined(DEBUG_FBM320) || defined(MSG_LOG)
		printf("%s: The version of sensor is unknown.\n", __func__);
#endif
		break;
	}
	return err;
}



static int32_t fbm320_set_oversampling_rate(struct fbm320_data *barom
        , enum fbm320_osr osr_setting)
{
	uint8_t reg_addr;
	uint8_t data_buf;

	barom->oversampling_rate = osr_setting;
#ifdef DEBUG_FBM320
	printf("%s:Setting of oversampling_rate:%#x\r\n", __func__, barom->oversampling_rate);
#endif

	/* Setting conversion time for pressure measurement */
	switch (osr_setting) {
	case osr_1024:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR1024;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_0;
		break;
	case osr_2048:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR2048;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_1;
		break;
	case osr_4096:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR4096;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_2;
		break;
	case osr_8192:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR8192;
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_3;
		break;
	case osr_16384:
		barom->cnvTime_press = FBM320_CONVERSION_usTIME_OSR16384;

		reg_addr = 0xa6;

		barom->bus_read(reg_addr, sizeof(uint8_t), &data_buf);

		data_buf &= 0xf8;
		data_buf |= 0x6;

//		barom->bus_write(reg_addr, sizeof(uint8_t), &data_buf);
		barom->cmd_start_p = FBM320_MEAS_PRESS_OVERSAMP_2;
		barom->bus_read(0xA6, sizeof(uint8_t), &data_buf);
#ifdef DEBUG_FBM320
		printf("reg_0xA6:%#x\n\r", data_buf);
#endif
		break;
	}
	/* Setting covversion time for temperature measurement */
	barom->cnvTime_temp = FBM320_CONVERSION_usTIME_OSR1024;

	return 0;
}
static int32_t fbm320_chipid_check(struct fbm320_data *barom)
{
	int32_t err;
	uint8_t chip_id_read;

	err = barom->bus_read(FBM320_CHIP_ID_REG, sizeof(uint8_t), &chip_id_read);
#ifdef DEBUG_FBM320
	printf("%s: chip_id reading is %#x \n", __func__, chip_id_read);
#endif

	if (chip_id_read != FBM320_CHIP_ID) {
		err = -1;
		return err;
	} else {
		barom->chip_id = chip_id_read;
		return err = 0;
	}
}



/**
 * @brief      { API for triggering measurement procedure and updating
 *               the temperature and pressure data in fbm320_data structure. }
 */
void fbm320_update_data(void)
{
	static uint32_t t_start_flag = 0;
	static uint32_t p_start_flag = 0;
	static uint32_t tick_current;
	static uint32_t tick_last;
	static uint32_t tick_diff;

	tick_current = HAL_GetTick(); //TMR0_Ticks;
	tick_diff = tick_current - tick_last;

	if (t_start_flag == 0 && !fbm320_update_rdy) {

#ifdef DEBUG_FBM320
		printf("start t_measurement\r\n");
#endif
		fbm320_startMeasure_temp(barom);
		t_start_flag = 1;
		tick_last = tick_current; //TMR0_Ticks;

	} else if ((tick_diff * 1000 > barom->cnvTime_temp ) && (p_start_flag == 0)) {

#ifdef DEBUG_FBM320
		printf("start p_measurement\r\n");
#endif
		fbm320_get_raw_temperature(barom);
		fbm320_startMeasure_press(barom);
		p_start_flag = 1;
		tick_last = tick_current; //TMR0_Ticks;
	} else if (tick_diff * 1000 > barom->cnvTime_press ) {
#ifdef DEBUG_FBM320
		printf("read pressure\r\n");
#endif
		fbm320_get_raw_pressure(barom);
		t_start_flag = 0;
		p_start_flag = 0;
		tick_current = 0;
		tick_last = 0;
//		TMR0_Ticks = 0;
		fbm320_update_rdy = 1;
	}
#ifdef DEBUG_FBM320
	printf("tick_current:%d\r\n", tick_current);
	printf("tick_last:%d\r\n", tick_last);
//	printf("FBM320 is updating %d\r\n", TMR0_Ticks);
#endif
	return ;
}



/**
 * @brief      { API for converting pressure value to altitude }
 *
 * @param[in]  pressure_input  The pressure_input is in unit of 0.125 Pa
 *
 * @return     { The altitude value is in unit millimeter(mm) }
 */
int32_t fbm325_get_altitude(int32_t pressure_input)
{
	return pressure_altitude_conversion(pressure_input);
}









#ifdef FBM_320
static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom) // 320
{
	struct fbm320_calibration_data *cali = &(barom->calibration);
	int32_t status;

	uint16_t R[10] = {0};
	uint8_t tmp[FBM320_CALIBRATION_DATA_LENGTH] = {0};

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START0,
	                         (FBM320_CALIBRATION_DATA_LENGTH - 2) * sizeof(uint8_t),
	                         (uint8_t *)tmp);

	if (status < 0)
		goto exit;

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START1, sizeof(uint8_t), (uint8_t *)tmp + 18 );

	if (status < 0)
		goto exit;

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START2, sizeof(uint8_t), (uint8_t *)tmp + 19);

	if (status < 0)
		goto exit;

	/* Read OTP data here */
	R[0] = (tmp[0] << 8 | tmp[1]);
	R[1] = (tmp[2] << 8 | tmp[3]);
	R[2] = (tmp[4] << 8 | tmp[5]);
	R[3] = (tmp[6] << 8 | tmp[7]);
	R[4] = (tmp[8] << 8 | tmp[9]);
	R[5] = (tmp[10] << 8 | tmp[11]);
	R[6] = (tmp[12] << 8 | tmp[13]);
	R[7] = (tmp[14] << 8 | tmp[15]);
	R[8] = (tmp[16] << 8 | tmp[17]);
	R[9] = (tmp[18] << 8 | tmp[19]);

	/* Coefficient reconstruction */
	cali->C0 = R[0] >> 4;
	cali->C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	cali->C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	cali->C3 = R[2] >> 3;
	cali->C5 = R[4] >> 1;
	cali->C6 = R[5] >> 3;
	cali->C8 = R[7] >> 3;
	cali->C9 = R[8] >> 2;
	cali->C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	cali->C11 = R[9] & 0xFF;
	switch (barom->hw_ver) {
	case hw_ver_b1:
		cali->C4 = ((uint32_t)R[3] << 1) | (R[5] & 1);
		cali->C7 = ((uint32_t)R[6] << 2) | ((R[0] >> 2) & 3);
		cali->C12 = ((R[5] & 6) << 2) | (R[7] & 7);
		break;
	case hw_ver_b2:
	case hw_ver_b3:
	case hw_ver_b4:
	default:
		cali->C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
		cali->C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
		cali->C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);
		break;
	};

#ifdef DEBUG_FBM320
	printf("%s: R0= %#x\n", DEVICE_NAME, R[0]);
	printf("%s: R1= %#x\n", DEVICE_NAME, R[1]);
	printf("%s: R2= %#x\n", DEVICE_NAME, R[2]);
	printf("%s: R3= %#x\n", DEVICE_NAME, R[3]);
	printf("%s: R4= %#x\n", DEVICE_NAME, R[4]);
	printf("%s: R5= %#x\n", DEVICE_NAME, R[5]);
	printf("%s: R6= %#x\n", DEVICE_NAME, R[6]);
	printf("%s: R7= %#x\n", DEVICE_NAME, R[7]);
	printf("%s: R8= %#x\n", DEVICE_NAME, R[8]);
	printf("%s: R9= %#x\n", DEVICE_NAME, R[9]);
	printf("%s: C0= %d\n", DEVICE_NAME, cali->C0);
	printf("%s: C1= %d\n", DEVICE_NAME, cali->C1);
	printf("%s: C2= %d\n", DEVICE_NAME, cali->C2);
	printf("%s: C3= %d\n", DEVICE_NAME, cali->C3);
	printf("%s: C4= %d\n", DEVICE_NAME, cali->C4);
	printf("%s: C5= %d\n", DEVICE_NAME, cali->C5);
	printf("%s: C6= %d\n", DEVICE_NAME, cali->C6);
	printf("%s: C7= %d\n", DEVICE_NAME, cali->C7);
	printf("%s: C8= %d\n", DEVICE_NAME, cali->C8);
	printf("%s: C9= %d\n", DEVICE_NAME, cali->C9);
	printf("%s: C10= %d\n", DEVICE_NAME, cali->C10);
	printf("%s: C11= %d\n", DEVICE_NAME, cali->C11);
	printf("%s: C12= %d\n", DEVICE_NAME, cali->C12);
#endif
exit:
	return status;
}



int32_t fbm320_calculation(struct fbm320_data *barom) // 320
{
	struct fbm320_calibration_data *cali = &barom->calibration;
	int32_t X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32;
	int32_t PP1, PP2, PP3, PP4, CF;
	int32_t RT, RP, UT, UP, DT, DT2;

	/* calculation for real temperature value*/
	UT = barom->raw_temperature;
	DT = ((UT - 8388608) >> 4) + (cali->C0 << 4);
	switch (barom->hw_ver) {
	case hw_ver_b1:
		X01 = (cali->C1 + 4418L) * DT >> 1;
		break;
	case hw_ver_b2:
	case hw_ver_b3:
	case hw_ver_b4:
	default:
		X01 = (cali->C1 + 4459L) * DT >> 1;
		break;
	};
	X02 = ((((cali->C2 - 256L) * DT) >> 14) * DT) >> 4;
	X03 = (((((cali->C3 * DT) >> 18) * DT) >> 18) * DT);
	DT2 = (X01 + X02 + X03) >> 12;
	RT =  ((2500L << 15) - X01 - X02 - X03) >> 15;
	/* calculation for real pressure value*/
	UP = barom->raw_pressure;
	switch (barom->hw_ver) {
	case hw_ver_b1:
		X11 = (cali->C5 * DT2);
		break;
	case hw_ver_b2:
	case hw_ver_b3:
	case hw_ver_b4:
	default:
		X11 = ((cali->C5 - 4443L) * DT2);
		break;
	};

	X12 = (((cali->C6 * DT2) >> 16) * DT2) >> 2;
	switch (barom->hw_ver) {
	case hw_ver_b1:
		X13 = ((X11 + X12) >> 10) + ((cali->C4 + 211288) << 4);
		X21 = ((cali->C8 + 7209L) * DT2) >> 10;
		break;
	case hw_ver_b2:
	case hw_ver_b3:
	case hw_ver_b4:
	default:
		X13 = ((X11 + X12) >> 10) + ((cali->C4 + 120586) << 4);
		X21 = ((cali->C8 + 7180L) * DT2) >> 10;
		break;
	};

	X22 = (((cali->C9 * DT2) >> 17) * DT2) >> 12;
	X23 = (X22 >= X21) ? (X22 - X21) : (X21 - X22);
	switch (barom->hw_ver) {
	case hw_ver_b1:
		X24 = (X23 >> 11) * (cali->C7 + 285594);
		X25 = ((X23 & 0x7FF) * (cali->C7 + 285594)) >> 11;
		if ((X22 - X21) < 0)
			X26 = ((0 - X24 - X25) >> 11) + cali->C7 + 285594;
		else
			X26 = ((X24 + X25) >> 11) + cali->C7 + 285594;
		break;
	case hw_ver_b2:
	case hw_ver_b3:
	case hw_ver_b4:
	default:
		X24 = (X23 >> 11) * (cali->C7 + 166426);
		X25 = ((X23 & 0x7FF) * (cali->C7 + 166426)) >> 11;
		if ((X22 - X21) < 0)
			X26 = ((0 - X24 - X25) >> 11) + cali->C7 + 166426;
		else
			X26 = ((X24 + X25) >> 11) + cali->C7 + 166426;
		break;
	};

	PP1 = ((UP - 8388608) - X13) >> 3;
	PP2 = (X26 >> 11) * PP1;
	PP3 = ((X26 & 0x7FF) * PP1) >> 11;
	PP4 = (PP2 + PP3) >> 10;
	CF = (2097152 + cali->C12 * DT2) >> 3;
	X31 = (((CF * cali->C10) >> 17) * PP4) >> 2;
	X32 = (((((CF * cali->C11) >> 15) * PP4) >> 18) * PP4);
	RP = ((X31 + X32) >> 15) + PP4 + 100000;

	barom->real_temperature = RT; //uint:0.01 degree Celsius
	barom->real_pressure = RP; //uint: Pa

#ifdef DEBUG_FBM320
	printf("%s: calibrated pressure: %d\n", DEVICE_NAME, RP);
#endif

	return 0;
}



/**
// * @brief      { This function is used for converting pressure value to altitude.
// *               The standard sea-level pressure is 1013.25 hPa. }
// *
// * @param[in]  real_pressure  The real pressure is in unit of 0.125 Pa
// *
// * @return     { The altitude value is in unit of millimeter(mm) }
// */
static int32_t pressure_altitude_conversion(int32_t real_pressure) // 325 & 320
{
	int32_t RP, h0, hs0, HP1, HP2, RH;
	int16_t hs1, dP0;
	int8_t P0;

	RP = real_pressure;

	if ( RP >= 824000 ) {
		P0	=	103	;
		h0	=	-138507	;
		hs0	=	-5252	;
		hs1	=	311	;
	} else if ( RP >= 784000 ) {
		P0	=	98	;
		h0	=	280531	;
		hs0	=	-5468	;
		hs1	=	338	;
	} else if ( RP >= 744000 ) {
		P0	=	93	;
		h0	=	717253	;
		hs0	=	-5704	;
		hs1	=	370	;
	} else if ( RP >= 704000 ) {
		P0	=	88	;
		h0	=	1173421	;
		hs0	=	-5964	;
		hs1	=	407	;
	} else if ( RP >= 664000 ) {
		P0	=	83	;
		h0	=	1651084	;
		hs0	=	-6252	;
		hs1	=	450	;
	} else if ( RP >= 624000 ) {
		P0	=	78	;
		h0	=	2152645	;
		hs0	=	-6573	;
		hs1	=	501	;
	} else if ( RP >= 584000 ) {
		P0	=	73	;
		h0	=	2680954	;
		hs0	=	-6934	;
		hs1	=	560	;
	} else if ( RP >= 544000 ) {
		P0	=	68	;
		h0	=	3239426	;
		hs0	=	-7342	;
		hs1	=	632	;
	} else if ( RP >= 504000 ) {
		P0	=	63	;
		h0	=	3832204	;
		hs0	=	-7808	;
		hs1	=	719	;
	} else if ( RP >= 464000 ) {
		P0	=	58	;
		h0	=	4464387	;
		hs0	=	-8345	;
		hs1	=	826	;
	} else if ( RP >= 424000 ) {
		P0	=	53	;
		h0	=	5142359	;
		hs0	=	-8972	;
		hs1	=	960	;
	} else if ( RP >= 384000 ) {
		P0	=	48	;
		h0	=	5874268	;
		hs0	=	-9714	;
		hs1	=	1131	;
	} else if ( RP >= 344000 ) {
		P0	=	43	;
		h0	=	6670762	;
		hs0	=	-10609	;
		hs1	=	1354	;
	} else if ( RP >= 304000 ) {
		P0	=	38	;
		h0	=	7546157	;
		hs0	=	-11711	;
		hs1	=	1654	;
	} else if ( RP >= 264000 ) {
		P0	=	33	;
		h0	=	8520395	;
		hs0	=	-13103	;
		hs1	=	2072	;
	} else {
		P0	=	28	;
		h0	=	9622536	;
		hs0	=	-14926	;
		hs1	=	2682	;
	}

	dP0	=	RP - P0 * 8000;
	HP1	=	( hs0 * dP0 ) >> 1;
	HP2	=	((( hs1 * dP0 ) >> 14 ) * dP0 ) >> 4;
	RH	=	(( HP1 + HP2 ) >> 8 ) + h0;

	return RH;
}
#endif // ifdef FBM_320





#ifdef FBM_325
/**
 * @brief      { API for reading calibration data saved in OTP memory }
 *
 * @param      barom  FBM320 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom) // 325
{
    struct fbm320_calibration_data *cali = &(barom->calibration);

    int32_t status;
	uint16_t R[10] = {0};

	uint8_t tmp[FBM320_CALIBRATION_DATA_LENGTH] = {0};

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START0,
	                         (FBM320_CALIBRATION_DATA_LENGTH - 2) * sizeof(uint8_t),
	                         (uint8_t *)tmp);

	if (status < 0)
		goto exit;

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START1, sizeof(uint8_t), (uint8_t *)tmp + 18 );
	if (status < 0)
		goto exit;

	status = barom->bus_read(FBM320_CALIBRATION_DATA_START2, sizeof(uint8_t), (uint8_t *)tmp + 19);
	if (status < 0)
		goto exit;

	/* Read OTP data here */
	R[0] = (tmp[0] << 8 | tmp[1]);
	R[1] = (tmp[2] << 8 | tmp[3]);
	R[2] = (tmp[4] << 8 | tmp[5]);
	R[3] = (tmp[6] << 8 | tmp[7]);
	R[4] = (tmp[8] << 8 | tmp[9]);
	R[5] = (tmp[10] << 8 | tmp[11]);
	R[6] = (tmp[12] << 8 | tmp[13]);
	R[7] = (tmp[14] << 8 | tmp[15]);
	R[8] = (tmp[16] << 8 | tmp[17]);
	R[9] = (tmp[18] << 8 | tmp[19]);

	/* Coefficient reconstruction */
	cali->C0 = R[0] >> 3;
	cali->C1 = ((R[1] & (0xFF00)) >> 6 ) + ((R[2] & (0x30)) >> 4);
	cali->C2 = ((R[1] & (0xFF)) << 2 ) + ((R[2] & (0xC)) >> 2);
	cali->C3 = R[2] >> 6;
	cali->C4 = (R[3] << 3) + ((R[4] & (0x3)) << 1) + ((R[5] & (0x20)) >> 5);
	cali->C5 = R[4] >> 2;
	cali->C6 = R[5] >> 6;
	cali->C7 = (R[6] << 3) + (R[0] & (0x7));
	cali->C8 = R[7] >> 4;
	cali->C9 = ((R[8] & (0xFF00)) >> 6) + (R[2] & (0x3));
	cali->C10 = ((R[8] & (0xFF)) << 2) + ((R[7] & (0xC)) >> 2);
	cali->C11 = R[9] >> 8;
	cali->C12 = R[5] & (0x1F);
	cali->C13 = ((R[9] & (0xFF)) << 2) + (R[7] & (0x3));

#if defined(DEBUG_FBM320) || defined(MSG_LOG)
	printf("%s: R0= %#x\n", DEVICE_NAME, R[0]);
	printf("%s: R1= %#x\n", DEVICE_NAME, R[1]);
	printf("%s: R2= %#x\n", DEVICE_NAME, R[2]);
	printf("%s: R3= %#x\n", DEVICE_NAME, R[3]);
	printf("%s: R4= %#x\n", DEVICE_NAME, R[4]);
	printf("%s: R5= %#x\n", DEVICE_NAME, R[5]);
	printf("%s: R6= %#x\n", DEVICE_NAME, R[6]);
	printf("%s: R7= %#x\n", DEVICE_NAME, R[7]);
	printf("%s: R8= %#x\n", DEVICE_NAME, R[8]);
	printf("%s: R9= %#x\n", DEVICE_NAME, R[9]);
	printf("%s: C0= %d\n", DEVICE_NAME, cali->C0);
	printf("%s: C1= %d\n", DEVICE_NAME, cali->C1);
	printf("%s: C2= %d\n", DEVICE_NAME, cali->C2);
	printf("%s: C3= %d\n", DEVICE_NAME, cali->C3);
	printf("%s: C4= %d\n", DEVICE_NAME, cali->C4);
	printf("%s: C5= %d\n", DEVICE_NAME, cali->C5);
	printf("%s: C6= %d\n", DEVICE_NAME, cali->C6);
	printf("%s: C7= %d\n", DEVICE_NAME, cali->C7);
	printf("%s: C8= %d\n", DEVICE_NAME, cali->C8);
	printf("%s: C9= %d\n", DEVICE_NAME, cali->C9);
	printf("%s: C10= %d\n", DEVICE_NAME, cali->C10);
	printf("%s: C11= %d\n", DEVICE_NAME, cali->C11);
	printf("%s: C12= %d\n", DEVICE_NAME, cali->C12);
	printf("%s: C13= %d\n", DEVICE_NAME, cali->C13);
#endif//DEBUG_FBM320

	cali->C0 -= 2220;
	cali->C1 += 5028;
	cali->C2 -= 512;
	cali->C3 = (cali->C3 - 512) * (-1);
	cali->C4 -= 127927;
	cali->C5 -= 9011;
	cali->C6 -= 554;
	cali->C7 += 206606;
	cali->C8 += 9226;
	cali->C9 -= 388;
	cali->C10 -= 814;
	cali->C11 -= 227;
	cali->C12 -= 17;
	cali->C13 -= 688;

exit:
	return status;
}



/**
 * @brief      { API for calculating real temperature and pressure values.
 *               The results are stored in fbm320_data structure.
 *               "barom->real_temperature" is represented real temperature value.
 *               "barom->real_temperature" is in uint of 0.01 drgree Celsius.
 *               "barom->real_pressure" is represented real pressure value.
 *               "barom->real_pressure" is in unit of Pa. }
 *
 * @param      barom  pointer of fbm320 data structure
 *
 * @return     { description_of_the_return_value }
 */
int32_t fbm320_calculation(struct fbm320_data *barom) // 325
{
	struct fbm320_calibration_data *cali = &barom->calibration;
	int32_t DT, DT1, DP, DP1, DP2, DP3, DP4, DP5, CF, RT, RP, UT, UP, DT1A, DT1B;
	int32_t	DPC1, DPC2;

	/* calculation for real temperature value*/
	UT = barom->raw_temperature - 8388608;
	DT = ( UT >> 6 ) + ( cali->C0 << 2 );
	DT1	= ( cali->C1 * DT ) >> 1;
	DT1	= ((((( cali->C2 * DT ) >> 15 ) * DT ) >> 1 ) - DT1 ) >> 3;
	DT1A = ((( cali->C3 * DT ) >>15 ) * DT ) >>5;
	DT1B = (( abs ( DT1A )) >>13 ) * DT;
	DT1B = (((( abs ( DT1A )) &(0x1FFF)) * DT ) >>13 ) + DT1B	;
		if( DT1A <0 )
	DT1B = 0 - DT1B;
	DT1 = ( DT1B + DT1 ) >>8;
	/* calculation for real pressure value*/
	UP = barom->raw_pressure - 8388608;
	DP = ( UP >> 4 ) - cali->C4;
	DP1	= ( cali->C7 >> 11 ) * DP;
	DP = ((( cali->C7 & (0x7FF) ) * DP ) >> 11 ) + DP1;
	DP1	= cali->C5 * DT1;
	DP1	= (((( cali->C6 * DT1 ) >> 8 ) * DT1 ) >> 5 ) + DP1;
	DP1	= ((((((( cali->C13 * DT1 ) >> 9 ) * DT1 ) >> 15 ) * DT1 ) >> 4 ) + DP1 ) >> 5;
	DP = ( DP - DP1 ) >> 8;
	DP2	= cali->C8 * DT1;
	DP2	= (((((( cali->C9 * DT1 ) >> 8 ) * DT1 ) >> 5 ) + DP2 ) >> 8 ) + 8388608;
	DP3	= (( abs ( DP2 )) >> 12 ) * DP;
	DP = (((( abs ( DP2 ) & (0xFFF) ) * DP ) >> 12 ) + DP3 ) >> 8;
	CF = cali->C12 * DT1 + 1048576;
	DP3	= (( DP >> 1 ) * cali->C10 ) >> 3;
	DP4	= ( cali->C11 * DP ) >> 13;
	DP5	= (( abs ( DP4 )) >> 9 ) * DP;
	DP5	= (((( abs ( DP4 ) & (0x1FF) ) * DP ) >> 9 ) + DP5 ) >> 1;
	if ( DP4 > 0 ) DP3 = ( DP3 + DP5 ) >> 12;
	else DP3 = ( DP3 - DP5 ) >> 12;
	DP4	= (( abs ( CF )) >> 11 ) * DP3;
	DP4	= (((( abs ( CF ) & (0x7FF) ) * DP3 ) >> 11 ) + DP4 ) >> 9;
	DP = ( DP - DP4 ) >> 2;
	RT = (DT1 >> 2) + 2500;
	RP = DP + 800000;

	DT = RT - 2000;
	DP = 3000 * DT;
	if ( DT < 0 ) {
		DPC1 = -3141;
		DPC2 = -3269;
		DP = (((( DPC2 * DT ) >> 9 ) * DT ) >> 2 ) + DP;
		DP = (((((DPC1 * DT ) >> 9 ) * DT ) >> 13 ) * DT + DP ) >> 15;
	}
	else {
		DPC1 = 3403;
		DPC2 = -2828;
		DP = (((( DPC2 * DT ) >> 9 ) * DT ) >> 2 ) + DP;
		DP = (((((((DPC1 * DT ) >> 9 ) * DT ) >> 13 ) * DT ) >> 3 ) + DP ) >> 15;
	}
	RP = RP - DP;

	barom->real_temperature = RT; //uint:0.01 degree Celsius
	barom->real_pressure = RP; //uint:0.125 Pa

#ifdef DEBUG_FBM320
	printf("%s: calibrated pressure: %d\n", DEVICE_NAME, RP);
#endif//DEBUG_FBM320

	return 0;
}



/**
// * @brief      { This function is used for converting pressure value to altitude.
// *               The standard sea-level pressure is 1013.25 hPa. }
// *
// * @param[in]  real_pressure  The real pressure is in unit of 0.125 Pa
// *
// * @return     { The altitude value is in unit of millimeter(mm) }
// */
static int32_t pressure_altitude_conversion(int32_t real_pressure) // 325 & 320
{
	int32_t RP, h0, hs0, HP1, HP2, RH;
	int16_t hs1, dP0;
	int8_t P0;

	RP = real_pressure;

	if ( RP >= 824000 ) {
		P0	=	103	;
		h0	=	-138507	;
		hs0	=	-5252	;
		hs1	=	311	;
	} else if ( RP >= 784000 ) {
		P0	=	98	;
		h0	=	280531	;
		hs0	=	-5468	;
		hs1	=	338	;
	} else if ( RP >= 744000 ) {
		P0	=	93	;
		h0	=	717253	;
		hs0	=	-5704	;
		hs1	=	370	;
	} else if ( RP >= 704000 ) {
		P0	=	88	;
		h0	=	1173421	;
		hs0	=	-5964	;
		hs1	=	407	;
	} else if ( RP >= 664000 ) {
		P0	=	83	;
		h0	=	1651084	;
		hs0	=	-6252	;
		hs1	=	450	;
	} else if ( RP >= 624000 ) {
		P0	=	78	;
		h0	=	2152645	;
		hs0	=	-6573	;
		hs1	=	501	;
	} else if ( RP >= 584000 ) {
		P0	=	73	;
		h0	=	2680954	;
		hs0	=	-6934	;
		hs1	=	560	;
	} else if ( RP >= 544000 ) {
		P0	=	68	;
		h0	=	3239426	;
		hs0	=	-7342	;
		hs1	=	632	;
	} else if ( RP >= 504000 ) {
		P0	=	63	;
		h0	=	3832204	;
		hs0	=	-7808	;
		hs1	=	719	;
	} else if ( RP >= 464000 ) {
		P0	=	58	;
		h0	=	4464387	;
		hs0	=	-8345	;
		hs1	=	826	;
	} else if ( RP >= 424000 ) {
		P0	=	53	;
		h0	=	5142359	;
		hs0	=	-8972	;
		hs1	=	960	;
	} else if ( RP >= 384000 ) {
		P0	=	48	;
		h0	=	5874268	;
		hs0	=	-9714	;
		hs1	=	1131	;
	} else if ( RP >= 344000 ) {
		P0	=	43	;
		h0	=	6670762	;
		hs0	=	-10609	;
		hs1	=	1354	;
	} else if ( RP >= 304000 ) {
		P0	=	38	;
		h0	=	7546157	;
		hs0	=	-11711	;
		hs1	=	1654	;
	} else if ( RP >= 264000 ) {
		P0	=	33	;
		h0	=	8520395	;
		hs0	=	-13103	;
		hs1	=	2072	;
	} else {
		P0	=	28	;
		h0	=	9622536	;
		hs0	=	-14926	;
		hs1	=	2682	;
	}

	dP0	=	RP - P0 * 8000;
	HP1	=	( hs0 * dP0 ) >> 1;
	HP2	=	((( hs1 * dP0 ) >> 14 ) * dP0 ) >> 4;
	RH	=	(( HP1 + HP2 ) >> 8 ) + h0;

	return RH;
}
#endif // #ifdef FBM_325





#ifdef FBM_GIT

static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom) // git
{
	int32_t _result;

	struct fbm320_calibration_data *cali = &(barom->calibration);

	uint16_t R[10] = {0};
	uint8_t _read_buffer[2] = {0};
	uint8_t _parameter_index = 0;


	for (_parameter_index = 0; _parameter_index < 9; _parameter_index++) {
		_result = barom->bus_read(FBM320_CALIBRATION_DATA_START0 + (_parameter_index * 2), 1, &_read_buffer[0]);
		if (_result < 0)
			goto exit;

		_result = barom->bus_read(FBM320_CALIBRATION_DATA_START1 + (_parameter_index * 2), 1, &_read_buffer[1]);
		if (_result < 0)
			goto exit;

		R[_parameter_index] = ((uint8_t)_read_buffer[0] << 8 | _read_buffer[1]);
	} // for (_parameter_index = 0; _parameter_index < 9; _parameter_index++) {

	_result = barom->bus_read(FBM320_CALIBRATION_DATA_START2, 1, &_read_buffer[0]);
	if (_result < 0)
		goto exit;

	_result = barom->bus_read(FBM320_CALIBRATION_DATA_START3, 1, &_read_buffer[1]);
	if (_result < 0)
		goto exit;

	R[9] = ((uint8_t)_read_buffer[0] << 8 | _read_buffer[1]);

	/* Coefficient reconstruction */
	cali->C0 = R[0] >> 4;
	cali->C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	cali->C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	cali->C3 = R[2] >> 3;
	cali->C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
	cali->C5 = R[4] >> 1;
	cali->C6 = R[5] >> 3;
	cali->C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
	cali->C8 = R[7] >> 3;
	cali->C9 = R[8] >> 2;
	cali->C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	cali->C11 = R[9] & 0xFF;
	cali->C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);
exit:
	return _result;
} // static int32_t fbm320_read_store_otp_data(struct fbm320_data *barom)



int32_t fbm320_calculation(struct fbm320_data *barom)	// git
{
	int32_t _result = 0;

	struct fbm320_calibration_data *cali = &barom->calibration;
	int32_t X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32;
	int32_t PP1, PP2, PP3, PP4, CF;
	int32_t RT, RP, UT, UP, DT, DT2;

	/* calculation for real temperature value*/
	UT = barom->raw_temperature;
	DT = ((UT - 8388608) >> 4) + (cali->C0 << 4);
	X01 = (cali->C1 + 4459) * DT >> 1;
	X02 = ((((cali->C2 - 256) * DT) >> 14) * DT) >> 4;
	X03 = (((((cali->C3 * DT) >> 18) * DT) >> 18) * DT);
	RT = ((2500 << 15) - X01 - X02 - X03) >> 15;

	DT2 = (X01 + X02 + X03) >> 12;
	X11 = ((cali->C5 - 4443) * DT2);
	X12 = (((cali->C6 * DT2) >> 16) * DT2) >> 2;
	X13 = ((X11 + X12) >> 10) + ((cali->C4 + 120586) << 4);

	X21 = ((cali->C8 + 7180) * DT2) >> 10;
	X22 = (((cali->C9 * DT2) >> 17) * DT2) >> 12;
	X23 = abs(X22 - X21);
	X24 = (X23 >> 11) * (cali->C7 + 166426);
	X25 = ((X23 & 0x7FF) * (cali->C7 + 166426)) >> 11;
	X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + cali->C7 + 166426) : (((X24 + X25) >> 11) + cali->C7 + 166426);

	UP = barom->raw_pressure;
	PP1 = ((UP - 8388608) - X13) >> 3;
	PP2 = (X26 >> 11) * PP1;
	PP3 = ((X26 & 0x7FF) * PP1) >> 11;
	PP4 = (PP2 + PP3) >> 10;

	CF = (2097152 + cali->C12 * DT2) >> 3;
	X31 = (((CF * cali->C10) >> 17) * PP4) >> 2;
	X32 = (((((CF * cali->C11) >> 15) * PP4) >> 18) * PP4);
	RP = ((X31 + X32) >> 15) + PP4 + 99880;

	barom->real_temperature = RT; //uint:0.01 degree Celsius
	barom->real_pressure = RP;	//uint: Pa

	return _result;
}



/**
 * @brief      { This function is used for converting pressure value to altitude.
 *               The standard sea-level pressure is 1013.25 hPa. }
 *
 * @param[in]  real_pressure  The real pressure is in unit of 0.125 Pa
 *
 * @return     { The altitude value is in unit of millimeter(mm) }
 */
static int32_t pressure_altitude_conversion(int32_t real_pressure) // git
{
	int8_t P0 = 0;
	int16_t hs1 = 0;
	int16_t dP0 = 0;
	int32_t RP = 0;
	int32_t h0 = 0;
	int32_t hs0 = 0;
	int32_t HP1 = 0;
	int32_t HP2 = 0;
	int32_t RH = 0;

	RP = real_pressure;

	if (RP >= 103000) {
		P0 = 103;
		h0 = -138507;
		hs0 = -21007;
		hs1 = 311;
	} else if (RP >= 98000) {
		P0 = 98;
		h0 = 280531;
		hs0 = -21869;
		hs1 = 338;
	} else if (RP >= 93000) {
		P0 = 93;
		h0 = 717253;
		hs0 = -22813;
		hs1 = 370;
	} else if (RP >= 88000) {
		P0 = 88;
		h0 = 1173421;
		hs0 = -23854;
		hs1 = 407;
	} else if (RP >= 83000) {
		P0 = 83;
		h0 = 1651084;
		hs0 = -25007;
		hs1 = 450;
	} else if (RP >= 78000) {
		P0 = 78;
		h0 = 2152645;
		hs0 = -26292;
		hs1 = 501;
	} else if (RP >= 73000) {
		P0 = 73;
		h0 = 2680954;
		hs0 = -27735;
		hs1 = 560;
	} else if (RP >= 68000) {
		P0 = 68;
		h0 = 3239426;
		hs0 = -29366;
		hs1 = 632;
	} else if (RP >= 63000) {
		P0 = 63;
		h0 = 3832204;
		hs0 = -31229;
		hs1 = 719;
	} else if (RP >= 58000) {
		P0 = 58;
		h0 = 4464387;
		hs0 = -33377;
		hs1 = 826;
	} else if (RP >= 53000) {
		P0 = 53;
		h0 = 5142359;
		hs0 = -35885;
		hs1 = 960;
	} else if (RP >= 48000) {
		P0 = 48;
		h0 = 5874268;
		hs0 = -38855;
		hs1 = 1131;
	} else if (RP >= 43000) {
		P0 = 43;
		h0 = 6670762;
		hs0 = -42434;
		hs1 = 1354;
	} else if (RP >= 38000) {
		P0 = 38;
		h0 = 7546157;
		hs0 = -46841;
		hs1 = 1654;
	} else if (RP >= 33000) {
		P0 = 33;
		h0 = 8520395;
		hs0 = -52412;
		hs1 = 2072;
	} else {
		P0 = 28;
		h0 = 9622536;
		hs0 = -59704;
		hs1 = 2682;
	}

	dP0 = RP - P0 * 1000;
	HP1 = (hs0 * dP0) >> 2;
	HP2 = (((hs1 * dP0) >> 10) * dP0) >> 4;
	RH = ((h0 << 6) + HP1 + HP2) >> 6;

	return RH;
}
#endif // ifdef FBM_GIT



