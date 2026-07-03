/*
 * Zephyr fatal error hook - captures CPU fault details to retained RAM
 * before the system reboots.
 *
 * This file is compiled WITHOUT LTO (-fno-lto) to avoid weak symbol
 * resolution conflicts with Zephyr kernel's __weak k_sys_fatal_error_handler.
 * See CMakeLists.txt for the set_source_files_properties() call.
 */
#include <zephyr/fatal.h>
#include <zephyr/fatal_types.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

#include "power.h"
#include "../retained.h"

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	/* ARM Cortex-M fault status registers (fixed addresses in SCB) */
	volatile uint32_t *scb_cfsr = (volatile uint32_t *)0xE000ED28u;
	volatile uint32_t *scb_hfsr = (volatile uint32_t *)0xE000ED2Cu;

	uint32_t cfsr_val = *scb_cfsr;
	uint32_t hfsr_val = *scb_hfsr;

	/*
	 * Save fault details to retained RAM before the pending reset destroys
	 * all context. retained memory survives system resets (NVIC_SystemReset).
	 */
	if (retained && retained->fatal_error_info.magic == FATAL_ERROR_INFO_MAGIC) {
		retained->fatal_error_info.reason = reason;
		retained->fatal_error_info.cfsr = cfsr_val;
		retained->fatal_error_info.hfsr = hfsr_val;

		if (esf != NULL) {
			retained->fatal_error_info.pc  = esf->basic.pc;
			retained->fatal_error_info.lr  = esf->basic.lr;
		} else {
			retained->fatal_error_info.pc = 0;
			retained->fatal_error_info.lr = 0;
		}
	}

	/* Synchronous output to UART/RTT before the reboot */
	printk("\n*** ZEPHYR FATAL ERROR ***\n");
	printk("Reason: %u\n", reason);
	if (esf != NULL) {
		printk("PC:    0x%08X\n", esf->basic.pc);
		printk("LR:    0x%08X\n", esf->basic.lr);
	}
	printk("CFSR:  0x%08X\n", cfsr_val);
	printk("HFSR:  0x%08X\n", hfsr_val);

	/* Busy-wait to give UART/RTT time to flush */
	for (volatile uint32_t d = 0; d < 2000000; d++) {
		__asm__ volatile("nop");
	}

	/* Trigger system reset (standard CONFIG_RESET_ON_FATAL_ERROR path) */
	sys_arch_reboot(0);
	CODE_UNREACHABLE;
}
