/**
 * @file ethernet_phy.c
 * @author cy023
 * @date 2022.08.21
 * @brief 
 * 
 */

#include "assert.h"
#include "config_phy.h"
#include "ethernet_phy.h"
#include "ethernet_mac.h"

void mac_write_phy_reg(uint16_t addr, uint16_t reg, uint16_t val);
void mac_read_phy_reg(uint16_t addr, uint16_t reg, uint16_t *val);

/*******************************************************************************
 * Read / Write, Set / Clear Operation
 ******************************************************************************/

void ethernet_phy_read_reg(uint16_t reg, uint16_t *val)
{
    ASSERT((reg <= 0x1F) && val);
    mac_read_phy_reg(CONF_ETHERNET_PHY_ADDRESS, reg, val);
}

void ethernet_phy_write_reg(uint16_t reg, uint16_t val)
{
    ASSERT((reg <= 0x1F));
    mac_write_phy_reg(CONF_ETHERNET_PHY_ADDRESS, reg, val);
}

void ethernet_phy_set_reg_bit(uint16_t reg, uint16_t offset)
{
    uint16_t val;
    ASSERT((reg <= 0x1F));
    mac_read_phy_reg(CONF_ETHERNET_PHY_ADDRESS, reg, &val);
    val |= offset;
    mac_write_phy_reg(CONF_ETHERNET_PHY_ADDRESS, reg, val);
}

void ethernet_phy_clear_reg_bit(uint16_t reg, uint16_t offset)
{
    uint16_t val;
    ASSERT((reg <= 0x1F));
    mac_read_phy_reg(CONF_ETHERNET_PHY_ADDRESS, reg, &val);
    val &= ~offset;
    mac_write_phy_reg(CONF_ETHERNET_PHY_ADDRESS, reg, val);
}

/*******************************************************************************
 * Basic Operation
 ******************************************************************************/

void ethernet_phy_init(void)
{
    ethernet_phy_write_reg(ETH_PHY_BMCR, CONF_ETHERNET_PHY_CONTROL_BMCR);
}

void ethernet_phy_set_powerdown(bool state)
{
    if (state)
        ethernet_phy_set_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_POWER_DOWN_MASK);
    else
        ethernet_phy_clear_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_POWER_DOWN_MASK);
}

void ethernet_phy_set_isolate(bool state)
{
    if (state)
        ethernet_phy_set_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_ISOLATE_MASK);
    else
        ethernet_phy_clear_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_ISOLATE_MASK);
}

void ethernet_phy_restart_autoneg(void)
{
    ethernet_phy_set_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_RESTART_AUTONEG_MASK);
}

void ethernet_phy_set_loopback(bool state)
{
    if (state)
        ethernet_phy_set_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_LOOPBACK_MASK);
    else
        ethernet_phy_clear_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_LOOPBACK_MASK);
}

void ethernet_phy_get_link_status(bool *status)
{
    uint16_t val;
    mac_read_phy_reg(CONF_ETHERNET_PHY_ADDRESS, ETH_PHY_BMSR, &val);
    *status = (val & ETH_PHY_BMSR_LINK_STATUS_MASK) ? true : false;
}

void ethernet_phy_reset(void)
{
    ethernet_phy_set_reg_bit(ETH_PHY_BMCR, ETH_PHY_BMCR_RESET_MASK);
}
