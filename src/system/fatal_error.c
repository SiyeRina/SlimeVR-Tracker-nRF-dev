/*
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/arch/exception.h>
#include <zephyr/fatal_types.h>
#include "retained.h"

#ifdef CONFIG_ARM
#include <zephyr/arch/arm/arch.h>
#endif

/* Human-readable names for Zephyr fatal error reason codes */
static const char *fatal_reason_name(unsigned int reason)
{
	switch (reason) {
	case K_ERR_CPU_EXCEPTION:   return "CPU exception";
	case K_ERR_SPURIOUS_IRQ:    return "Spurious interrupt";
	case K_ERR_STACK_CHK_FAIL:  return "Stack overflow";
	case K_ERR_KERNEL_OOPS:     return "Kernel oops";
	case K_ERR_KERNEL_PANIC:    return "Kernel panic (assertion failure)";
#ifdef CONFIG_ARM
	case K_ERR_ARM_MEM_GENERIC:
	case K_ERR_ARM_MEM_STACKING:
	case K_ERR_ARM_MEM_UNSTACKING:
	case K_ERR_ARM_MEM_DATA_ACCESS:
	case K_ERR_ARM_MEM_INSTRUCTION_ACCESS:
	case K_ERR_ARM_MEM_FP_LAZY_STATE_PRESERVATION:
		return "Memory manager fault";

	case K_ERR_ARM_BUS_GENERIC:
	case K_ERR_ARM_BUS_STACKING:
	case K_ERR_ARM_BUS_UNSTACKING:
	case K_ERR_ARM_BUS_PRECISE_DATA_BUS:
	case K_ERR_ARM_BUS_IMPRECISE_DATA_BUS:
	case K_ERR_ARM_BUS_INSTRUCTION_BUS:
	case K_ERR_ARM_BUS_FP_LAZY_STATE_PRESERVATION:
		return "Bus fault";

	case K_ERR_ARM_USAGE_GENERIC:
	case K_ERR_ARM_USAGE_DIV_0:
	case K_ERR_ARM_USAGE_UNALIGNED_ACCESS:
	case K_ERR_ARM_USAGE_STACK_OVERFLOW:
	case K_ERR_ARM_USAGE_NO_COPROCESSOR:
	case K_ERR_ARM_USAGE_ILLEGAL_EXC_RETURN:
	case K_ERR_ARM_USAGE_ILLEGAL_EPSR:
	case K_ERR_ARM_USAGE_UNDEFINED_INSTRUCTION:
		return "Usage fault";

	case K_ERR_ARM_SECURE_GENERIC:
	case K_ERR_ARM_SECURE_ENTRY_POINT:
	case K_ERR_ARM_SECURE_INTEGRITY_SIGNATURE:
	case K_ERR_ARM_SECURE_EXCEPTION_RETURN:
	case K_ERR_ARM_SECURE_ATTRIBUTION_UNIT:
	case K_ERR_ARM_SECURE_TRANSITION:
	case K_ERR_ARM_SECURE_LAZY_STATE_PRESERVATION:
	case K_ERR_ARM_SECURE_LAZY_STATE_ERROR:
		return "Security fault";

	case K_ERR_ARM_UNDEFINED_INSTRUCTION:
	case K_ERR_ARM_ALIGNMENT_FAULT:
	case K_ERR_ARM_BACKGROUND_FAULT:
	case K_ERR_ARM_PERMISSION_FAULT:
	case K_ERR_ARM_PERMISSION_FAULT_2ND_LEVEL:
	case K_ERR_ARM_SYNC_EXTERNAL_ABORT:
	case K_ERR_ARM_ASYNC_EXTERNAL_ABORT:
	case K_ERR_ARM_SYNC_PARITY_ERROR:
	case K_ERR_ARM_ASYNC_PARITY_ERROR:
	case K_ERR_ARM_DEBUG_EVENT:
	case K_ERR_ARM_TRANSLATION_FAULT:
	case K_ERR_ARM_TRANSLATION_FAULT_2ND_LEVEL:
	case K_ERR_ARM_UNSUPPORTED_EXCLUSIVE_ACCESS_FAULT:
	case K_ERR_ARM_ACCESS_FLAG_FAULT_1ST_LEVEL:
	case K_ERR_ARM_ACCESS_FLAG_FAULT_2ND_LEVEL:
	case K_ERR_ARM_CACHE_MAINTENANCE_INSTRUCTION_FAULT:
	case K_ERR_ARM_DOMAIN_FAULT_1ST_LEVEL:
	case K_ERR_ARM_DOMAIN_FAULT_2ND_LEVEL:
	case K_ERR_ARM_SYNC_EXTERNAL_ABORT_TRANSLATION_TABLE_1ST_LEVEL:
	case K_ERR_ARM_SYNC_EXTERNAL_ABORT_TRANSLATION_TABLE_2ND_LEVEL:
	case K_ERR_ARM_TLB_CONFLICT_ABORT:
	case K_ERR_ARM_SYNC_PARITY_ERROR_TRANSLATION_TABLE_1ST_LEVEL:
	case K_ERR_ARM_SYNC_PARITY_ERROR_TRANSLATION_TABLE_2ND_LEVEL:
		return "Hard fault / other ARM fault";
#endif
	default:
		return "Unknown";
	}
}

/**
 * @brief Custom Zephyr fatal error handler
 *
 * Overrides Zephyr's weak k_sys_fatal_error_handler to capture fatal error
 * details before the system reboots. This catches assertion failures,
 * kernel panics, CPU exceptions, and stack overflows that would otherwise
 * cause a silent SREQ reset with no diagnostic information.
 *
 * The error details are saved to retained RAM so they survive the reset
 * and can be displayed by the `lastreset` shell command on next boot.
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	const char *name = fatal_reason_name(reason);
	uint32_t pc = 0;

	/* Extract program counter from exception stack frame if available */
	if (esf != NULL) {
		pc = esf->basic.pc;
	}

	/* Save to retained RAM for retrieval after reboot */
	if (retained != NULL && retained->last_reset_info.magic == LAST_RESET_INFO_MAGIC) {
		retained->fatal_error_info.reason = reason;
		retained->fatal_error_info.pc = pc;
		retained->fatal_error_info.magic = FATAL_ERROR_INFO_MAGIC;
	}

	/* Print fatal error information via printk (RTT + UART).
	 * Using printk instead of LOG because the logging subsystem
	 * may be in an inconsistent state during a fatal error. */
	printk("\n=== ZEPHYR FATAL ERROR ===\n");
	printk("Reason: %u (%s)\n", reason, name);
	if (esf != NULL && pc != 0) {
		printk("PC:     0x%08X\n", pc);
		printk("LR:     0x%08X\n", (unsigned int)esf->basic.lr);
		printk("R0:     0x%08X\n", (unsigned int)esf->basic.a1);
		printk("xPSR:   0x%08X\n", (unsigned int)esf->basic.xpsr);
	}
	printk("System will reboot...\n");
	printk("===========================\n\n");

	/* Brief busy-wait to allow RTT/UART to flush the output.
	 * We use k_busy_wait instead of k_sleep because the scheduler
	 * may not be functional during a fatal error handler. */
	k_busy_wait(200000);  /* 200 ms */

	/* After this function returns, Zephyr will call sys_arch_reboot()
	 * if CONFIG_RESET_ON_FATAL_ERROR=y, causing a system reset.
	 * The fatal error info will be available via `lastreset` after reboot. */
}
