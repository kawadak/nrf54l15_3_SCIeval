/*
 * coder         : kawadak
 * date          : 2026-05-18
 * description   : SCI test application main
 */

#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log.h>

#include "mysci.h"

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void) {
        
	int err;

	console_init();

	LOG_INF("Starting Bluetooth Shorter Connection Intervals sample");

	err = mysci_init();
	if (err) {
		LOG_ERR("mysci_init failed (%d)", err);
		return 0;
	}

	mysci_select_mode();

	mysci_start();

	while (1) {
		mysci_run();
	}

	return 0;
}