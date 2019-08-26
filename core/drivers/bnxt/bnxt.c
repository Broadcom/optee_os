// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2019 Broadcom.
 */

#include <drivers/bcm-include/bnxt.h>
#include <initcall.h>
#include <io.h>
#include <kernel/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define BNXT_REG_CTRL_BASE 0x3040000
#define BNXT_REG_ECO_RESERVED 0x3042400
#define BNXT_FLASH_ACCESS_DONE_BIT 2
#define NIC400_BNXT_IDM_IO_CONTROL_DIRECT 0x60e00408
#define BNXT_INDIRECT_BASE 0x60800000
#define BNXT_INDIRECT_ADDR_MASK 0x3fffffu
#define BNXT_INDIRECT_BASE_MASK (~BNXT_INDIRECT_ADDR_MASK)
#define BNXT_INDIRECT_WINDOW_SIZE (BNXT_INDIRECT_ADDR_MASK + 1)
#define BNXT_REG_CTRL_BPE_MODE_REG 0x0
#define BNXT_REG_CTRL_BPE_MODE_FASTBOOT_MODE_BIT 2
#define BNXT_REG_CTRL_BPE_MODE_CM3_RST_BIT 1
#define BNXT_REG_CTRL_BPE_STAT_REG 0x4
#define BNXT_REG_CTRL_FSTBOOT_PTR_REG 0x8
#define BNXT_ERROR_MASK 0xf0000000
#define BNXT_CTRL_ADDR(x) (BNXT_REG_CTRL_BASE + (x))
#define BNXT_HANDSHAKE_TIMEOUT_MS 10000

#define KONG_REG_CTRL_MODE_REG 0x03900000
#define KONG_REG_CTRL_MODE_CPUHALT_N_BIT 0

#define BNXT_STICKY_BYTE_POR 0x04030088
#define BNXT_STICKY_BYTE_POR_MHB_BIT 4

static vaddr_t bnxt_access_window_virt_addr;
static vaddr_t bnxt_indirect_dest_addr;

static void bnxt_prepare_access_window(uint32_t addr)
{
	addr &= BNXT_INDIRECT_BASE_MASK;
	io_write32(bnxt_access_window_virt_addr, addr);
}

static vaddr_t bnxt_indirect_tgt_addr(uint32_t addr)
{
	addr &= BNXT_INDIRECT_ADDR_MASK;
	return (vaddr_t)(bnxt_indirect_dest_addr + addr);
}

uint32_t bnxt_write32_multiple(uintptr_t dst,
			       uintptr_t src,
			       uint32_t num_entries,
			       int src_4byte_increment)
{
	uint32_t i;
	vaddr_t target;

	if (num_entries == 0)
		return 0;

	/* only write up to the next window boundary */
	if ((dst & BNXT_INDIRECT_BASE_MASK) !=
	    ((dst + num_entries * sizeof(uint32_t)) & BNXT_INDIRECT_BASE_MASK))
		num_entries = (((dst + BNXT_INDIRECT_WINDOW_SIZE) &
				BNXT_INDIRECT_BASE_MASK) -
			       dst) /
			      sizeof(uint32_t);

	bnxt_prepare_access_window((uint32_t)dst);
	target = bnxt_indirect_tgt_addr((uint32_t)dst);
	for (i = 0; i < num_entries; i++) {
		io_write32(target, *(uint32_t *)src);
		target += sizeof(uint32_t);
		src += (src_4byte_increment * sizeof(uint32_t));
	}

	return num_entries;
}

static uint32_t bnxt_read(uint32_t addr)
{
	bnxt_prepare_access_window(addr);
	return io_read32(bnxt_indirect_tgt_addr(addr));
}

static uint32_t bnxt_read_ctrl(uint32_t offset)
{
	return bnxt_read(BNXT_CTRL_ADDR(offset));
}

static void bnxt_write(uint32_t addr, uint32_t value)
{
	bnxt_prepare_access_window(addr);
	io_write32(bnxt_indirect_tgt_addr(addr), value);
}


static void bnxt_write_ctrl(uint32_t offset, uint32_t value)
{
	bnxt_write(BNXT_CTRL_ADDR(offset), value);
}

static int bnxt_handshake_done(void)
{
	uint32_t value;

	value = bnxt_read(BNXT_REG_ECO_RESERVED);
	value &= BIT(BNXT_FLASH_ACCESS_DONE_BIT);

	return value != 0;
}

int bnxt_wait_handshake(void)
{
	uint32_t timeout = BNXT_HANDSHAKE_TIMEOUT_MS;
	uint32_t status;

	DMSG("Waiting for ChiMP handshake...\n");
	do {
		if (bnxt_handshake_done())
			break;
		/* No need to wait if ChiMP reported an error */
		status = bnxt_read_ctrl(BNXT_REG_CTRL_BPE_STAT_REG);
		if (status & BNXT_ERROR_MASK) {
			DMSG("ChiMP error 0x%x. Wait aborted\n", status);
			break;
		}
		mdelay(1);
	} while (--timeout);

	if (!bnxt_handshake_done()) {
		if (timeout == 0)
			DMSG("Timeout waiting for ChiMP handshake\n");
	} else {
		DMSG("Got handshake from ChiMP!\n");
	}
	return bnxt_handshake_done();
}

void bnxt_chimp_halt(void)
{
	uint32_t value;

	value = bnxt_read_ctrl(BNXT_REG_CTRL_BPE_MODE_REG);
	value |= BIT(BNXT_REG_CTRL_BPE_MODE_CM3_RST_BIT);
	bnxt_write_ctrl(BNXT_REG_CTRL_BPE_MODE_REG, value);
}

void bnxt_kong_halt(void)
{
	uint32_t value;

	value = bnxt_read(KONG_REG_CTRL_MODE_REG);
	value &= ~BIT(KONG_REG_CTRL_MODE_CPUHALT_N_BIT);
	bnxt_write(KONG_REG_CTRL_MODE_REG, value);
}

int bnxt_fastboot(uintptr_t addr)
{
	uint32_t value;

	value = bnxt_read(BNXT_STICKY_BYTE_POR);
	value |= BIT(BNXT_STICKY_BYTE_POR_MHB_BIT);
	bnxt_write(BNXT_STICKY_BYTE_POR, value);

	/* Set the fastboot address and type */
	bnxt_write_ctrl(BNXT_REG_CTRL_FSTBOOT_PTR_REG, addr);

	/* Set fastboot mode & take BNXT CPU1 out of reset */
	value = bnxt_read_ctrl(BNXT_REG_CTRL_BPE_MODE_REG);
	value |= BIT(BNXT_REG_CTRL_BPE_MODE_FASTBOOT_MODE_BIT);
	value &= ~BIT(BNXT_REG_CTRL_BPE_MODE_CM3_RST_BIT);
	bnxt_write_ctrl(BNXT_REG_CTRL_BPE_MODE_REG, value);

	return 0;
}

static TEE_Result bnxt_init(void)
{
	bnxt_access_window_virt_addr = (vaddr_t)(phys_to_virt((paddr_t)
					NIC400_BNXT_IDM_IO_CONTROL_DIRECT,
					MEM_AREA_IO_SEC));
	bnxt_indirect_dest_addr = (vaddr_t)(phys_to_virt((paddr_t)
					    BNXT_INDIRECT_BASE,
					    MEM_AREA_IO_SEC));
	return TEE_SUCCESS;
}
driver_init(bnxt_init);
