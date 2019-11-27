// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2019, Broadcom
 */

#include <drivers/bcm_gpio.h>
#include <drivers/pl022_spi.h>
#include <io.h>
#include <kernel/pseudo_ta.h>
#include <kernel/tee_time.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <platform_config.h>
#include <string.h>
#include <trace.h>

#define SPI_SERVICE_UUID \
		{ 0x6272636D, 0x2018, 0x1101,  \
		{ 0x42, 0x43, 0x4D, 0x5F, 0x53, 0x50, 0x49, 0x30 } }

/*
 * Following commands are for SPI FLASH device.
 * This can be extended to the other SPI Slaves
 */
enum pta_bcm_spi_cmd {
	/*
	 * Configure SPI
	 *
	 * [in]    value[0].a:    spi mode
	 * [in]    value[0].b:    spi speed
	 * [in]    value[1].a:    data size
	 * [in]    value[1].b:    loopback
	 * [in]    value[2].a:    gpio pin for cs
	 */
	PTA_BCM_SPI_CMD_CFG = 0,

	/*
	 * Read identification Register
	 */
	PTA_BCM_SPI_CMD_RDID,

	/*
	 * Erase complete chip
	 */
	PTA_BCM_SPI_CMD_ERASE,

	/*
	 * Read 256B data from flash starting from 0x0
	 */
	PTA_BCM_SPI_CMD_READ,

	/*
	 * Write 256B data to flash starting from 0x0
	 */
	PTA_BCM_SPI_CMD_WRITE,

	/*
	 * Verify SPI Flash for erase, write and read ops
	 */
	PTA_BCM_SPI_CMD_FLASH_TEST,
};

#define PIN_MUX_FUNC_MASK		0x7
#define PIN_MUX_FUNC_GPIO		0x3

/* SPI Flash commands */
#define MACRONIX_FLASH_WT		0x02 /* Write Data */
#define MACRONIX_FLASH_RD		0x03 /* Read Data */
#define MACRONIX_FLASH_WD		0x04 /* Write Disable*/
#define MACRONIX_FLASH_RDSR		0x05 /* Read Status Register */
#define MACRONIX_FLASH_WE		0x06 /* Write Enable */
#define MACRONIX_FLASH_CE		0xC7 /* Chip Erase */
#define MACRONIX_FLASH_RDID		0x9F /* Read Identification */

/* Write in Progress bit */
#define MACRONIX_FLASH_RDSR_WIP_BIT	0x1

/* Buffer size for read and write operations */
#define DATA_BUFF_SZ			256

/* Time needed to prepare the data after sending the command */
#define SPI_FLASH_DATA_PREP_MS		20
#define SPI_FLASH_INCREMENT_RD_MS	10
#define SPI_FLASH_CMD_COMPLETE_MS	200
#define SPI_FLASH_CS_LOW_MS		2

/* MAX Iteration count for the command success check */
#define SPI_FLASH_CMD_ITER_CNT		100

/* Flash commands data sizes */
#define SPI_FLASH_ERASE_CMD_SZ		2
#define SPI_FLASH_RDID_SZ		3

#define SPI_TA_NAME		"pta_bcm_spi.ta"

struct pl022_data pd;
uint8_t tx_buff[DATA_BUFF_SZ];
uint8_t rx_buff[DATA_BUFF_SZ];

static void set_spi_cs_mux(void)
{
	vaddr_t virt_addr = (vaddr_t)phys_to_virt(SPI_0_CS_MUX_PAD,
						  MEM_AREA_IO_SEC);

	io_clrsetbits32(virt_addr, PIN_MUX_FUNC_MASK, PIN_MUX_FUNC_GPIO);
}

static void set_spi_cs(enum gpio_level value)
{
	struct bcm_gpio_chip *bcm_gc = NULL;
	struct gpio_chip *gc = NULL;
	uint32_t gpio_num = pd.cs_data.gpio_data.pin_num;

	bcm_gc = bcm_gpio_pin_to_chip(gpio_num);
	if (!bcm_gc) {
		EMSG("GPIO %u not supported", gpio_num);
		return;
	}

	gc = &bcm_gc->chip;

	/*
	 * For setting a value to GPIO Pin,
	 * make sure the PIN is configured in output direction.
	 */
	if (gc->ops->get_direction(gpio_num) != GPIO_DIR_OUT) {
		EMSG("gpio pin %u is configured as INPUT", gpio_num);
		return;
	}

	gc->ops->set_value(gpio_num, value);

	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
}

static TEE_Result read_rdsr_reg(void)
{
	uint8_t tx = 0;
	uint8_t rx = 0;
	uint8_t temp = 0;
	int timeout = SPI_FLASH_CMD_ITER_CNT;
	enum spi_result res = SPI_OK;

	/*
	 * Read the Read Status Register (RDSR)
	 * Need to wait till "Write in Progress"(WIP) bit is 0
	 */
	tx = MACRONIX_FLASH_RDSR;

	do {
		set_spi_cs(GPIO_LEVEL_LOW);
		res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
		if (res) {
			EMSG("Flash read RDSR cmd failed, err %d", res);
			return TEE_ERROR_GENERIC;
		}
		res = pd.chip.ops->txrx8(&pd.chip, &temp, &rx, 1);
		if (res) {
			EMSG("Flash read RDSR data failed, err %d", res);
			return TEE_ERROR_GENERIC;
		}
		set_spi_cs(GPIO_LEVEL_HIGH);
	} while ((rx & MACRONIX_FLASH_RDSR_WIP_BIT) && timeout--);

	if (timeout == 0) {
		EMSG("Timeout for flash read RDSR");
		return TEE_ERROR_GENERIC;
	}

	return TEE_SUCCESS;
}

static TEE_Result pta_spi_config(uint32_t param_types,
				 TEE_Param params[TEE_NUM_PARAMS])
{
	struct bcm_gpio_chip *bcm_gc = NULL;
	struct gpio_chip *gc = NULL;
	vaddr_t spi_base = 0;
	uint32_t mode = 0;
	uint32_t speed = 0;
	uint32_t data_size = 0;
	uint32_t loopback = 0;
	uint32_t gpio_num = 0;
	uint32_t exp_param_types = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						   TEE_PARAM_TYPE_VALUE_INPUT,
						   TEE_PARAM_TYPE_VALUE_INPUT,
						   TEE_PARAM_TYPE_NONE);

	if (exp_param_types != param_types) {
		EMSG("Invalid Param types");
		return TEE_ERROR_BAD_PARAMETERS;
	}

	spi_base = (vaddr_t)phys_to_virt(SPI_0_BASE, MEM_AREA_IO_SEC);

	/* Valid modes are 0, 1, 2 and 3 */
	mode = params[0].value.a & 0x3;
	speed = params[0].value.b;

	if (speed > SPI_0_CLK_HZ / 2) {
		EMSG("Bad Speed (%d) request", speed);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	data_size = params[1].value.a;
	loopback = params[1].value.b;

	/* Non loopback mode, configure GPIO */
	if (!loopback) {
		gpio_num = params[2].value.a;

		/* Configure the CS pin to GPIO */
		set_spi_cs_mux();

		bcm_gc = bcm_gpio_pin_to_chip(gpio_num);
		if (!bcm_gc) {
			EMSG("GPIO %u not supported", gpio_num);
			return TEE_ERROR_GENERIC;
		}

		gc = &bcm_gc->chip;

		iproc_gpio_set_secure(gpio_num);

		/* Set GPIO to output with default value to HIGH */
		gc->ops->set_direction(gpio_num, GPIO_DIR_OUT);
		gc->ops->set_value(gpio_num, GPIO_LEVEL_HIGH);

		/* Chip control is done manually */
		pd.cs_control = PL022_CS_CTRL_MANUAL;

		pd.cs_data.gpio_data.pin_num = gpio_num;
	} else {
		EMSG("Currently only Non-loopback mode supported");
		return TEE_ERROR_BAD_PARAMETERS;
	}

	/* Macronix Flash supports only 8bit and 16bit data size */
	if (data_size != 8 && data_size != 16) {
		EMSG("Macronix flash supports 8bit & 16bit data_sz");
		return TEE_ERROR_BAD_PARAMETERS;
	}

	pd.base = spi_base;
	pd.clk_hz = SPI_0_CLK_HZ;
	pd.speed_hz = speed;
	pd.mode = mode;
	pd.data_size_bits = data_size;
	pd.loopback = loopback;

	/* Initialize SPI controller */
	pl022_init(&pd);
	pd.chip.ops->configure(&pd.chip);
	pd.chip.ops->start(&pd.chip);

	return TEE_SUCCESS;
}

/*
 * Macronix Flash Read identification.
 * RDID instruction is for reading the manufacturer ID of 1-byte
 * and followed by Device ID of 2-byte. Macronix Manufacturer ID is
 * 0xC2(hex), memory type ID is 0x20(hex) as the first-byte device ID,
 * and individual device ID of second-byte ID is 10(hex) for MX25L512E.
 */
static TEE_Result pta_spi_rdid(void)
{
	uint8_t tx = 0;
	uint8_t rx[SPI_FLASH_RDID_SZ] = {0};
	enum spi_result res = SPI_OK;
	uint32_t i = 0;

	if (!pd.base) {
		EMSG("SPI is not initialized");
		return TEE_ERROR_BAD_STATE;
	}

	tx = MACRONIX_FLASH_RDID;

	set_spi_cs(GPIO_LEVEL_LOW);
	res = pd.chip.ops->txrx8(&pd.chip, &tx, rx, 1);
	if (res) {
		EMSG("Sending RDID failed, err %d", res);
		return TEE_ERROR_COMMUNICATION;
	}

	/*
	 * As we are operating at low speed, give enough time for the slave
	 * to prepare data.
	 */
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	tx = 0;
	for (i = 0; i < SPI_FLASH_RDID_SZ; i++) {
		res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx[i], 1);
		if (res) {
			EMSG("Reading RDID failed, err %d", res);
			return TEE_ERROR_COMMUNICATION;
		}
	}

	set_spi_cs(GPIO_LEVEL_HIGH);

	for (i = 0; i < SPI_FLASH_RDID_SZ; i++)
		IMSG("RDID [%d] = 0x%02x", i, rx[i]);

	return TEE_SUCCESS;
}

static TEE_Result pta_spi_erase(void)
{
	uint8_t tx = 0;
	uint8_t rx = 0;
	enum spi_result res = SPI_OK;

	if (!pd.base) {
		EMSG("SPI is not initialized");
		return TEE_ERROR_BAD_STATE;
	}

	/* Write enable operation */
	tx = MACRONIX_FLASH_WE;

	set_spi_cs(GPIO_LEVEL_LOW);
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash write enable failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	set_spi_cs(GPIO_LEVEL_HIGH);

	/* Chip Erase operation */
	tx = MACRONIX_FLASH_CE;

	set_spi_cs(GPIO_LEVEL_LOW);
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash chip erase failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	set_spi_cs(GPIO_LEVEL_HIGH);

	if (read_rdsr_reg() != TEE_SUCCESS)
		return TEE_ERROR_GENERIC;

	IMSG("Chip erase is successful");
	return TEE_SUCCESS;
}

static TEE_Result pta_spi_read(void)
{
	uint8_t tx = 0;
	uint8_t rx = 0;
	int i = 0;
	enum spi_result res = SPI_OK;

	if (!pd.base) {
		EMSG("SPI is not initialized");
		return TEE_ERROR_BAD_STATE;
	}

	if (read_rdsr_reg() != TEE_SUCCESS)
		return TEE_ERROR_GENERIC;

	/* Read Data operation */
	tx = MACRONIX_FLASH_RD;

	set_spi_cs(GPIO_LEVEL_LOW);
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash read data cmd failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	/* 24 bit Address */
	tx = 0x0;
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash send read addr failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash send read addr failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash send read addr failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	/* Reading Data from Flash */
	tx = 0x0;
	for (i = 0; i < DATA_BUFF_SZ; i++) {
		res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx_buff[i], 1);
		if (res) {
			EMSG("Flash read data failed, err %d", res);
			return TEE_ERROR_GENERIC;
		}
		tee_time_busy_wait(SPI_FLASH_INCREMENT_RD_MS);
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);
	set_spi_cs(GPIO_LEVEL_HIGH);

	for (i = 0; i < DATA_BUFF_SZ; ) {
		IMSG("%02x  %02x  %02x  %02x  %02x  %02x  %02x  %02x",
		     rx_buff[i], rx_buff[i + 1], rx_buff[i + 2],
		     rx_buff[i + 3], rx_buff[i + 4], rx_buff[i + 5],
		     rx_buff[i + 6], rx_buff[i + 7]);
		i = i + 8;
	}

	return TEE_SUCCESS;
}

static TEE_Result pta_spi_write(void)
{
	uint8_t tx = 0;
	uint8_t rx = 0;
	int i = 0;
	enum spi_result res = SPI_OK;

	if (!pd.base) {
		EMSG("SPI is not initialized");
		return TEE_ERROR_BAD_STATE;
	}

	/* Write enable operation */
	tx = MACRONIX_FLASH_WE;

	set_spi_cs(GPIO_LEVEL_LOW);
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash write enable cmd failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);
	set_spi_cs(GPIO_LEVEL_HIGH);

	/* Write Data operation */
	tx = MACRONIX_FLASH_WT;
	set_spi_cs(GPIO_LEVEL_LOW);
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash write data cmd failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	/* 24 bit Address */
	tx = 0x0;
	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash send write addr failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash send write addr failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}

	res = pd.chip.ops->txrx8(&pd.chip, &tx, &rx, 1);
	if (res) {
		EMSG("Flash send write addr failed, err %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	/* Fill incremental pattern in TX buff */
	for (i = 0; i < DATA_BUFF_SZ; i++)
		tx_buff[i] = i;

	/* Write Data to Flash */
	for (i = 0; i < DATA_BUFF_SZ; i++) {
		res = pd.chip.ops->txrx8(&pd.chip, &tx_buff[i], &rx, 1);
		if (res) {
			EMSG("Flash write data failed, err %d", res);
			return TEE_ERROR_GENERIC;
		}
		tee_time_busy_wait(SPI_FLASH_INCREMENT_RD_MS);
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);
	set_spi_cs(GPIO_LEVEL_HIGH);

	if (read_rdsr_reg() != TEE_SUCCESS)
		return TEE_ERROR_GENERIC;

	return TEE_SUCCESS;
}

/* Does erase, read, write and verify data */
static TEE_Result pta_spi_flash_test(void)
{
	TEE_Result res = TEE_SUCCESS;
	int i = 0;

	res = pta_spi_erase();
	if (res != TEE_SUCCESS) {
		EMSG("pta_spi_erase failed");
		return res;
	}

	res = pta_spi_write();
	if (res != TEE_SUCCESS) {
		EMSG("pta_spi_write failed");
		return res;
	}

	/* Fill Rx_buff with 0 */
	memset(rx_buff, 0, DATA_BUFF_SZ);

	res = pta_spi_read();
	if (res != TEE_SUCCESS) {
		EMSG("pta_spi_read failed");
		return res;
	}

	for (i = 0; i < DATA_BUFF_SZ; i++) {
		if (tx_buff[i] != rx_buff[i]) {
			IMSG("Data mismatch tx[%d] 0x%02x rx[%d] 0x%02x",
			     i, tx_buff[i], i, rx_buff[i]);
			res = TEE_ERROR_GENERIC;
		}
	}

	return res;
}

static TEE_Result invoke_command(void *session_context __unused,
				 uint32_t cmd_id,
				 uint32_t param_types,
				 TEE_Param params[TEE_NUM_PARAMS])
{
	TEE_Result res = TEE_SUCCESS;

	DMSG("command entry point[%d] for \"%s\"", cmd_id, SPI_TA_NAME);

	switch (cmd_id) {
	case PTA_BCM_SPI_CMD_CFG:
		res = pta_spi_config(param_types, params);
		break;
	case PTA_BCM_SPI_CMD_RDID:
		res = pta_spi_rdid();
		break;
	case PTA_BCM_SPI_CMD_ERASE:
		res = pta_spi_erase();
		break;
	case PTA_BCM_SPI_CMD_READ:
		res = pta_spi_read();
		break;
	case PTA_BCM_SPI_CMD_WRITE:
		res = pta_spi_write();
		break;
	case PTA_BCM_SPI_CMD_FLASH_TEST:
		res = pta_spi_flash_test();
		break;
	default:
		EMSG("cmd: %d Not supported %s", cmd_id, SPI_TA_NAME);
		res = TEE_ERROR_NOT_SUPPORTED;
		break;
	}

	return res;
}

pseudo_ta_register(.uuid = SPI_SERVICE_UUID,
		   .name = SPI_TA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .invoke_command_entry_point = invoke_command);
