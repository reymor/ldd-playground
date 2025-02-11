// SPDX-License-Identifier: GPL-2.0

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <uapi/linux/serial_reg.h>

/* IOCTL definitions (ideally declared in a header in uapi/linux/) */
#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER 1

struct serial_dev {
    void __iomem *regs;
    struct miscdevice miscdev;
    unsigned int counter;
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

static ssize_t serial_read(struct file *file, char __user *buf,
                           size_t sz, loff_t *ppos)
{
    return -EINVAL;
}

static ssize_t serial_write(struct file *fp, const char __user *buf,
                            size_t sz, loff_t *ppos)
{
    struct miscdevice *miscdev_ptr = fp->private_data;
    struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev,
                                             miscdev);
    int i;

    for (i = 0; i < sz; i++) {
        unsigned char c;

        if (get_user(c, buf + i))
            return -EFAULT;
        serial_write_char(serial, c);
        serial->counter++;

        if (c == '\n')
            serial_write_char(serial, '\r');
    }

    *ppos += sz;
    return sz;
}

static long serial_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *miscdev_ptr = fp->private_data;
    struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev, miscdev);
    unsigned int __user *argp = (unsigned int __user *)arg;

    switch (cmd) {
        case SERIAL_RESET_COUNTER:
            serial->counter = 0;
            break;
        case SERIAL_GET_COUNTER:
            if (put_user(serial->counter, argp))
                return -EFAULT;
            break;
        default:
            return -ENOTTY;
    }

    return 0;
}

static const struct file_operations serial_fops = {
    .owner = THIS_MODULE,
    .write = serial_write,
    .read = serial_read,
    .unlocked_ioctl = serial_ioctl,
};

static int serial_probe(struct platform_device *pdev)
{
    int ret;
    struct serial_dev *serial;
    struct resource *res;
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

    // clear uart fifos
    reg_write(serial, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        ret = -EINVAL;
        goto disable_runtime_pm;
    }

    // declare misc device
    serial->miscdev.minor = MISC_DYNAMIC_MINOR;
    serial->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
                                          "serial-%x", res->start);
    serial->miscdev.fops = &serial_fops;
    serial->miscdev.parent = &pdev->dev;

    platform_set_drvdata(pdev, serial);

    // register the misc device
    ret = misc_register(&serial->miscdev);
    if (ret) {
        dev_err(&pdev->dev, "Cannot  register misc device (%d)\n", ret);
        goto disable_runtime_pm;
    }

	return 0;
disable_runtime_pm:
    pm_runtime_disable(&pdev->dev);

    return ret;
}

static int serial_remove(struct platform_device *pdev)
{
    struct serial_dev *serial = platform_get_drvdata(pdev);

	pr_info("Called %s\n", __func__);
    misc_deregister(&serial->miscdev);
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
