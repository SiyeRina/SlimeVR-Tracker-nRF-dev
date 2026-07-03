#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

void sys_interface_suspend(void);
void sys_interface_resume(void);

void sys_request_WOM(bool, bool);
void sys_request_system_off(bool);
void sys_request_system_reboot(bool);

#include "retained.h"

/**
 * @brief Tag the source of the next SREQ (software-requested reset).
 * Call this immediately before sys_request_system_reboot().
 */
static inline void lastreset_tag_sreq_source(enum sreq_source src)
{
	extern struct retained_data *retained;
	if (retained->last_reset_info.magic == LAST_RESET_INFO_MAGIC) {
		retained->last_reset_info.sreq_source = (uint8_t)src;
	}
}

bool vin_read(void);
bool vbus_read(void);

#endif
