// SPDX-License-Identifier: GPL-2.0

#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <uapi/linux/serial_reg.h>

struct serial_dev {
    void __iomem *regs;
};

static const struct of_device_id serial_of_match[] = {
    { .compatible = "myuart,serial"},
    { /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(of, serial_of_match);

static u32 reg_read(struct serial_dev *serial, unsigned int reg)
{
    return readl(serial->regs + (reg * 4));
}

static void reg_write(struct serial_dev *serial, u32 val, unsigned int reg)
{
    writel(val, serial->regs + (reg * 4));
}

static void serial_write_char(struct serial_dev *serial, unsigned char c)
{
    while ((reg_read(serial, UART_LSR) & UART_LSR_THRE) == 0)
        cpu_relax();

    reg_write(serial, c, UART_TX);
}

static int serial_probe(struct platform_device *pdev)
{
    int ret;
    struct serial_dev *serial;
    unsigned int uartclk, baud_divisor;

    pr_info("Called %s\n", __func__);

    serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
    if (!serial)
        return -ENOMEM;

    // get a virtual address for the device registers
    serial->regs = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(serial->regs))
        return PTR_ERR(serial->regs);

    //enable power mgm
    pm_runtime_enable(&pdev->dev);
    pm_runtime_get_sync(&pdev->dev);

    ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &uartclk);
    if (ret) {
        dev_err(&pdev->dev, "clock-frequency property not found in device tree\n");
        goto disable_runtime_pm;
    }

    baud_divisor = uartclk / 16 / 115200;
    reg_write(serial, 0x07, UART_OMAP_MDR1);
    reg_write(serial, 0x00, UART_LCR);
    reg_write(serial, UART_LCR_DLAB, UART_LCR);
    reg_write(serial, baud_divisor & 0xff, UART_DLL);
    reg_write(serial, (baud_divisor >> 8) & 0xFF, UART_DLM);
    reg_write(serial, UART_LCR_WLEN8, UART_LCR);
    reg_write(serial, 0x00, UART_OMAP_MDR1);

    // soft reset
    reg_write(serial, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);

    //write a char for testing
    serial_write_char(serial, 'T');

	return 0;
disable_runtime_pm:
    pm_runtime_disable(&pdev->dev);

    return ret;
}

static int serial_remove(struct platform_device *pdev)
{
	pr_info("Called %s\n", __func__);
    pm_runtime_disable(&pdev->dev);
    return 0;
}

static struct platform_driver serial_driver = {
        .driver = {
                .owner = THIS_MODULE,
                .name = "serial",
                .of_match_table = of_match_ptr(serial_of_match),
        },
        .probe = serial_probe,
        .remove = serial_remove,
};
module_platform_driver(serial_driver);

MODULE_LICENSE("GPL");