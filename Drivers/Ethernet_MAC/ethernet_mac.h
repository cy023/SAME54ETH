/**
 * @file mac.c
 * @author cy023
 * @date 2022.08.20
 * @brief 
 * 
 */

#ifndef MAC_H
#define MAC_H

/*******************************************************************************
 * Tool
 ******************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "../SAME54_DFP/same54p20a.h"

#include "../SAME54_BSP/hal/utils/include/err_codes.h"
#include "../SAME54_BSP/hal/utils/include/utils.h"

/*******************************************************************************
 * IRQ HPL
 ******************************************************************************/

struct _irq_descriptor {
	void (*handler)(void *parameter);
	void *parameter;
};

uint8_t _irq_get_current(void);
void _irq_disable(uint8_t n);
void _irq_set(uint8_t n);
void _irq_clear(uint8_t n);
void _irq_enable(uint8_t n);
void _irq_register(const uint8_t number, struct _irq_descriptor *const irq);

/*******************************************************************************
 * GMAC HPL
 ******************************************************************************/

typedef struct _mac_device _mac_device_t;

typedef void (*_mac_cb_t)(_mac_device_t *const dev);

typedef enum mac_cb_type {
	MAC_RECEIVE_CB, /*!< One or more frame been received */
	MAC_TRANSMIT_CB /*!< One or more frame been transmited */
} mac_cb_type_t;

typedef struct mac_filter {
	uint8_t mac[6];     /*!< Destination address */
	uint8_t tid[2];     /*!< Type ID, 0x0600 IP package */
	bool    tid_enable; /*!< Enable TID matching */
} mac_filter_t;

/*******************************************************************************
 * GMAC HAL
 ******************************************************************************/

typedef void (*mac_cb_t)(void *ptr);

typedef struct mac_callbacks {
	mac_cb_t receive;
	mac_cb_t transmit;
} mac_callbacks_t;

/* GMAC instance */
// extern GMAC_t GMAC_DESCR;

void mac_init(void);
void mac_deinit(void);
void mac_enable(void);
void mac_disable(void);

int32_t mac_write(uint8_t *buf, uint32_t len);
uint32_t mac_read(uint8_t *buf, uint32_t len);
uint32_t mac_read_len(void);

void mac_enable_irq(void);
void mac_disable_irq(void);

int32_t mac_register_callback(const enum mac_cb_type type, const mac_cb_t func);

void mac_set_filter(uint8_t index, struct mac_filter *filter);
void mac_set_filter_ex(uint8_t mac[6]);
void mac_write_phy_reg(uint16_t addr, uint16_t reg, uint16_t val);
void mac_read_phy_reg(uint16_t addr, uint16_t reg, uint16_t *val);

#endif /* MAC_H */
