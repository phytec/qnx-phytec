/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 * Copyright 2018, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include <stdbool.h>
#include <time.h>
#include <hw/nxp/imx8/sci/sci.h>
#include "startup.h"
#include "board.h"
#include "qxp_cpu.h"
#include "imx_lpi2c_drv.h"
#include "aarch64/mx8xp.h"
#include <aarch64/imx8_common/imx_gpio.h>
#include "aarch64/mx8x_smc_call.h"

/* Global startup data */
imx_startup_data_t startup_data;

/* Callout prototype */
extern struct callout_rtn reboot_imx;

/* Startup console */
extern void imx_init_console(imx_startup_data_t * startup_data, struct debug_device * debug_devices);

const struct callout_slot callouts[] = {
    {
        CALLOUT_SLOT(reboot, _imx)
    },
};

/* The USB-Serial is on UART_1 */
struct debug_device debug_devices[] = {
    {
        "mx1",
        /* Initialize defaults variable, min. length:
         * "0x5A060000^0.115200.80000000.16" */
        { "                                                                      ", },
        imx_init_lpuart, imx_lpuart_put_char,
        { &imx_lpuart_display_char, &imx_lpuart_poll_key, &imx_lpuart_break_detect, }
    },
};

#if IMX_PCA9557_INIT_ENABLED && IMX_I2C_INIT_ENABLED
/**
 *  Initialization of the Expander A and the Expander B.
 *
 * @param startup_data Pointer to the startup data..
 *
 * @return    Always 0.
 */
static int imx_init_pca9557(imx_startup_data_t * startup_data)
{
    uint8_t data[2];

    /* Disable reset of I2C I/O Expander A,B */
    out32(PORT_EXP_RST_GPIO + IMX_GPIO_DR, in32(PORT_EXP_RST_GPIO + IMX_GPIO_DR) | (1 << PORT_EXP_RST_PIN));
    out32(PORT_EXP_RST_GPIO + IMX_GPIO_GDIR, in32(PORT_EXP_RST_GPIO + IMX_GPIO_GDIR) | 1 << PORT_EXP_RST_PIN);
    /* Initialize I2C channel associated with port expanders */
    imx_lpi2c_init(IMX_LSIO_I2C1_BASE, startup_data->imx_i2c_clock[1], 100000, NULL);

    /*********************************** I2C switch ********************************************/
    /* Enable all I2C channels */
    data[0] = I2C_SWITCH_CONTROL_B3 | I2C_SWITCH_CONTROL_B2 | I2C_SWITCH_CONTROL_B1 | I2C_SWITCH_CONTROL_B0;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, I2C_SWITCH_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 1);

    /*********************************** EXPANDER A ********************************************/
    /*
     * If a bit in this register is set (written with logic 1), the corresponding port pin’s
     * polarity is inverted. If a bit in this register is cleared (written with logic 0), the
     * corresponding port pin’s original polarity is retained.
     */
    data[0] = EXP_POLARITY_INVERSION_REGISTER;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    imx_lpi2c_recv(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 1, 1);
    data[1] &= ~(EXP_A_CPU_ENET_PHY_RST_B | EXP_A_USB_TYPEC_EN | EXP_A_MIPI_DSI0_EN);
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);
    /*
     * If a bit in this register is set, the corresponding port pin is enabled as an input with high-impedance output driver.
     * If a bit in this register is cleared, the corresponding port pin is enabled as an output.
     */
    data[0] = EXP_CONFIGURATION_REGISTER;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    imx_lpi2c_recv(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 1, 1);
    data[1] &= ~(EXP_A_CPU_ENET_PHY_RST_B | EXP_A_USB_TYPEC_EN | EXP_A_MIPI_DSI0_EN); /* EXP_A_CPU_ENET_PHY_RST_B, EXP_A_USB_TYPEC_EN as output */
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);
    /*
     * This register reflects the outgoing logic levels of the pins defined as outputs by the
     * Configuration register.
     */
    data[0] = EXP_OUTPUT_PORT_REGISTER;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    imx_lpi2c_recv(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 1, 1);
    data[1] |= (EXP_A_CPU_ENET_PHY_RST_B | EXP_A_USB_TYPEC_EN | EXP_A_MIPI_DSI0_EN); /* Enable EXP_A_CPU_ENET_PHY_RST_B, EXP_A_USB_TYPEC_EN */
    /* Toggle reset */
    data[1] &= ~(EXP_A_PER_RST_B);
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);
    imx_usleep(500);
    /* Toggle reset */
    data[1] |= (EXP_A_PER_RST_B);
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_A_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);

    /*********************************** EXPANDER B ********************************************/
    /*
     * If a bit in this register is set (written with logic 1), the corresponding port pin’s
     * polarity is inverted. If a bit in this register is cleared (written with logic 0), the
     * corresponding port pin’s original polarity is retained.
     */
    data[0] = EXP_POLARITY_INVERSION_REGISTER;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    imx_lpi2c_recv(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 1, 1);
    data[1] &= ~( EXP_B_MIPI_DSI1_EN |
                  EXP_B_BB_GPIO_EXP1 |
                  EXP_B_BB_GPIO_EXP2 |
                  EXP_B_BB_GPIO_EXP3
                );
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);
    /*
     * If a bit in this register is set, the corresponding port pin is enabled as an input with high-impedance output driver.
     * If a bit in this register is cleared, the corresponding port pin is enabled as an output.
     */
    data[0] = EXP_CONFIGURATION_REGISTER;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    imx_lpi2c_recv(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 1, 1);
    data[1] &= ~( EXP_B_BB_USB_OTG1_PWR_ON |
                  EXP_B_MIPI_DSI1_EN |
                  EXP_B_BB_GPIO_EXP1 |
                  EXP_B_BB_GPIO_EXP2 |
                  EXP_B_BB_GPIO_EXP3 |
                  EXP_B_BB_AUDIN_RST_B
                );
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);
    /*
     * This register reflects the outgoing logic levels of the pins defined as outputs by the
     * Configuration register.
     */
    data[0] = EXP_OUTPUT_PORT_REGISTER;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    imx_lpi2c_recv(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 1, 1);
    /* Pins to be put to low level */
    data[1] &= ~(
                 EXP_B_BB_AUDIN_RST_B
                );
    /* Pins to be put to high level */
    data[1] |= ( EXP_B_BB_USB_OTG1_PWR_ON |
                 EXP_B_MIPI_DSI1_EN |
                 EXP_B_BB_GPIO_EXP1 |
                 EXP_B_BB_GPIO_EXP2 |
                 EXP_B_BB_GPIO_EXP3
               );
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);
    imx_usleep(500);
    /* Toggle CS42888 reset high */
    data[1] |= EXP_B_BB_AUDIN_RST_B;
    imx_lpi2c_send(IMX_LSIO_I2C1_BASE, EXP_B_ADDR, IMX_LPI2C_ADDR_7BIT, data, 2, 1);

    return 0;
}
#endif	/* if IMX_PCA9557_INIT_ENABLED && IMX_I2C_INIT_ENABLED */

#if IMX_PCA6416_INIT_ENABLED && IMX_I2C_INIT_ENABLED
/**
 *  Initialization of the CAN IO expander on the baseboard.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return    Always 0.
 */
static int imx_init_pca6416(imx_startup_data_t * startup_data)
{
    uint8_t data[3];

    /* Initialize I2C channel associated with CAN IO expander */
    imx_lpi2c_init(IMX_M40_I2C0_BASE, startup_data->imx_i2c_clock[1], 100000, NULL);

    /*********************************** CAN IO EXPANDER ***************************************/
    /*
     * If a bit in this register is set (written with logic 1), the corresponding port pin’s
     * polarity is inverted. If a bit in this register is cleared (written with logic 0), the
     * corresponding port pin’s original polarity is retained.
     */
    data[0] = EXP_POLARITY_INVERSION_PORT_0_REGISTER;
    imx_lpi2c_send(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    data[2] = 0xFF;
    imx_lpi2c_recv(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 2, 1);
    /* Not inverted polarity set for the following pins (0 value written to the corresponding bit) */
    data[1] &= ~(BB_CAN_EXP_CAN01_STB_B     |
                 BB_CAN_EXP_CAN01_EN        |
                 BB_CAN_EXP_CAN0_WAKE_3V3_B |
                 BB_CAN_EXP_CAN1_WAKE_3V3_B
                );
    imx_lpi2c_send(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, data, 3, 1);
    /*
     * If a bit in this register is set, the corresponding port pin is enabled as an input with high-impedance output driver.
     * If a bit in this register is cleared, the corresponding port pin is enabled as an output.
     */
    data[0] = EXP_CONFIGURATION_PORT_0_REGISTER;
    imx_lpi2c_send(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    data[2] = 0xFF;
    imx_lpi2c_recv(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 2, 1);
    /* Output direction set for the following pins (0 value written to the corresponding bit) */
    data[1] &= ~(BB_CAN_EXP_CAN01_STB_B     |
                 BB_CAN_EXP_CAN01_EN        |
                 BB_CAN_EXP_CAN0_WAKE_3V3_B |
                 BB_CAN_EXP_CAN1_WAKE_3V3_B
                );
    imx_lpi2c_send(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, data, 3, 1);
    /*
     * This register reflects the outgoing logic levels of the pins defined as outputs by the
     * Configuration register.
     */
    data[0] = EXP_OUTPUT_PORT_0_REGISTER;
    imx_lpi2c_send(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, data, 1, 0);
    data[1] = 0xFF;
    data[2] = 0xFF;
    imx_lpi2c_recv(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, &data[1], 2, 1);

    /* CAN01_STB_B and CAN01_EN signals set to high to high (0 value written to the corresponding bit) */
    data[1] |= (BB_CAN_EXP_CAN01_STB_B     |
                BB_CAN_EXP_CAN01_EN        |
                BB_CAN_EXP_CAN0_WAKE_3V3_B |
                BB_CAN_EXP_CAN1_WAKE_3V3_B
               );
    imx_lpi2c_send(IMX_M40_I2C0_BASE, EXP_CAN_ADDR, IMX_LPI2C_ADDR_7BIT, data, 3, 1);

    return 0;
}
#endif	/* if IMX_PCA6416_INIT_ENABLED && IMX_I2C_INIT_ENABLED */

/**
 *  Startup program executing out of RAM.
 *
 * 1. It gathers information about the system and places it in a structure
 *    called the system page. The kernel references this structure to
 *    determine everything it needs to know about the system. This structure
 *    is also available to user programs (read only if protection is on)
 *    via _syspage->.
 *
 * 2. It (optionally) turns on the MMU and starts the next program
 *    in the image file system.
 *
 * @param argc Count of the arguments supplied to the startup.
 * @param argv Array of pointers to the strings which are those arguments.
 * @param envv Environment variable.
 *
 * @return     Always 0.
 */
int main(int argc, char **argv, __attribute__((unused)) char **envv)
{
    int opt, options = 0;
    uint32_t scfw_build, scfw_commit;
    sc_err_t sc_status;
    uint32_t val;
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    uint64_t atf_commit = 0;
#endif

    /* Initialize global startup structure */
    memset(&startup_data, 0x00U, sizeof(startup_data));

    /* Open IPC channel */
    sc_status = sc_ipc_open(&startup_data.ipc, IMX_SCU_IPC_MU);
    if (sc_status != SC_ERR_NONE) {
        crash("Cannot open IPC channel. Returned error %s", sc_status2str(sc_status));
    }
    /* Initialize startup serial console */
    imx_init_console(&startup_data, debug_devices);
    /* Initialize debugging output */
    select_debug(debug_devices, sizeof(debug_devices));

    add_callout_array(callouts, sizeof(callouts));

    /* Get SCFW build info */
    sc_misc_build_info(startup_data.ipc, &scfw_build, &scfw_commit);
    kprintf("SCFW build version: %d, SCFW commit: %x\n", scfw_build, scfw_commit);

#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    atf_commit = imx_sec_firmware_psci(IMX_FSL_SIP_BUILDINFO, IMX_FSL_SIP_BUILDINFO_GET_COMMITHASH, 0x00, 0x00, 0x00);
    if (atf_commit != IMX_PSCI_NOT_SUPPORTED) {
        kprintf("ATF commit: %s\n", (char*)&atf_commit);
    }
#endif

    /* Common options that should be avoided are:
       "AD:F:f:I:i:K:M:N:o:P:R:S:Tvr:j:Z" */
    while ((opt = getopt(argc, argv, COMMON_OPTIONS_STRING "W")) != -1) {
        switch (opt) {
            case 'W':
                options |= IMX_WDOG_ENABLE;
                break;
            default:
                handle_common_option(opt);
                break;
        }
    }

    /* Get chip revision */
    startup_data.chip_rev = imx_get_chip_rev(startup_data.ipc);
    /* Get chip type */
    startup_data.chip_type = imx_get_chip_type(startup_data.ipc);
    /* Get i.MX8 chip variant - used for fused i.MX8QXP chip only */
    if (startup_data.chip_type == IMX_CHIP_TYPE_QUAD_X_PLUS) {
        (void)sc_misc_otp_fuse_read(startup_data.ipc, 6U, &val);
        if ((val & 0x0CU) == 0x0CU) {
            startup_data.imx8dual_type = TRUE;
            kprintf("i.MX8 Dual type detected \n");
        } else {
            kprintf("i.MX8 Quad type detected \n");
        }
    } else {
        kprintf("WARNING: Unknown ChipType!\n");
    }

    /* Enable WDT */
    if (options & IMX_WDOG_ENABLE) {
        imx_wdg_enable(startup_data.ipc);
    }

    /* Collect information on all free RAM in the system */
    imx_init_raminfo();

    /* Set CPU frequency */
    if (cpu_freq == 0) {
        cpu_freq = imx_get_cpu_clk(startup_data.ipc, SC_R_A35);
    }

    /* Remove RAM used by modules in the image */
    alloc_ram(shdr->ram_paddr, shdr->ram_size, 1);

    /* Initialize SMP */
    init_smp();

    /* Initialize MMU */
    if (shdr->flags1 & STARTUP_HDR_FLAGS1_VIRTUAL) {
        init_mmu();
        board_mmu_enable();
        board_alignment_check_disable();
    }

    /* Initialize the Interrupts related Information */
    init_intrinfo();

    /* Initialize clocks */
    if (imx_init_clocks(&startup_data) != 0) {
        sc_ipc_close(startup_data.ipc);
        crash("Initialization of peripheral clocks failed.");
    }

    /* Initialize the Timer related information */
    init_qtime();
    imx_timer_init();

    /* Init Cache Controller */
    init_cacheattr();

    /* Initialize the CPU related information */
    init_cpuinfo();

    /* Initialize the Hwinfo section of the Syspage */
    imx_init_hwinfo(&startup_data);

    if (startup_data.imx8dual_type) {
        add_typed_string(_CS_MACHINE, "NXP i.MX8DXP/DX CPU Board");
    } else {
        add_typed_string(_CS_MACHINE, "NXP i.MX8QXP CPU Board");
    }

    /* Initialize peripheral pads */
    if (imx_init_pads(&startup_data) != 0) {
        sc_ipc_close(startup_data.ipc);
        crash("Initialization of peripheral pads failed.");
    }
#if IMX_PCA9557_INIT_ENABLED && IMX_I2C_INIT_ENABLED
    if (imx_init_pca9557(&startup_data) != 0) {
        sc_ipc_close(startup_data.ipc);
        crash("Initialization of pca9557 expanders failed.");
    }
#endif
#if IMX_PCA6416_INIT_ENABLED && IMX_I2C_INIT_ENABLED
    if (imx_init_pca6416(&startup_data) != 0) {
        sc_ipc_close(startup_data.ipc);
        crash("Initialization of pca6416 CAN IO expander on the baseboard failed.");
    }
#endif

#if IMX_USB_INIT_ENABLED
    /* Init USB OTG1 */
    imx_usb2_otg1_host_init();
    /* Init USB3 HOST */
    if (sc_rm_is_resource_owned(startup_data.ipc, SC_R_USB_2) != 0) {
        imx_usb3_otg2_host_init();
    }
#endif

    /* ENET MAC address initialization */
    imx_init_enet_mac_addr(startup_data.ipc);

    /* Report peripheral clock frequencies */
    imx_dump_clocks(&startup_data);
    /* Close IPC channel */
    sc_ipc_close(startup_data.ipc);
    /*
     * Load bootstrap executables in the image file system and Initialize
     * various syspage pointers. This must be the _last_ initialization done
     * before transferring control to the next program.
     */
    init_system_private();

    board_mmu_disable();

    /*
     * This is handy for debugging a new version of the startup program.
     * Commenting this line out will save a great deal of code.
     */
    print_syspage();

    return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/main.c $ $Rev: 904597 $")
#endif
