// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>

struct nunchuk_dev {
    struct i2c_client *i2c_client;
};

static const struct of_device_id nunchuk_of_match[] = {
    { .compatible = "nintendo,nunchuk"},
    { /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(of, nunchuk_of_match);

static int nunchuck_read_registers(struct i2c_client *client, u8 *recv)
{
    int ret;
    u8 buf[1];

    // Wait the device to get ready for a read
    usleep_range(10000, 20000);

    buf[0] = 0x00;
    ret = i2c_master_send(client, buf, 1);
    if (ret < 0) {
        dev_err(&client->dev, "error writing to device (%d)\n", ret);
        return ret;
    }

    usleep_range(10000, 20000);

    //read the register
    ret = i2c_master_recv(client, recv, 6);
    if (ret < 0) {
        dev_err(&client->dev, "error reading to device (%d)\n", ret);
        return ret;
    }

    return ret;
}

static void nunchuk_poll(struct input_dev *input)
{
    u8 recv[6];
    int bx, by, zpressed, cpressed;

    struct nunchuk_dev *nunchuck = input_get_drvdata(input);
    struct i2c_client *client = nunchuck->i2c_client;

    if (nunchuck_read_registers(client, recv) < 0)
        return;

    zpressed = (recv[5] & BIT(0)) ? 0 : 1;
    cpressed = (recv[5] & BIT(1)) ? 0 : 1;
    bx = recv[0];
    by = recv[1];

    input_report_key(input, BTN_Z, zpressed);
    input_report_key(input, BTN_C, cpressed);

    input_report_abs(input, ABS_X, bx);
    input_report_abs(input, ABS_Y, by);

    input_sync(input);
}

static int nunchuk_probe(struct i2c_client *client)
{
    char buf[2];
    u8 recv[6];
    struct input_dev *input;
    struct nunchuk_dev *nunchuk;
    int ret = 0;

    nunchuk = devm_kzalloc(&client->dev, sizeof(*nunchuk), GFP_KERNEL);
    if (!nunchuk)
        return -ENOMEM;

    //init the device
    buf[0] = 0x40;
    buf[1] = 0x00;
    ret = i2c_master_send(client, buf, 2);
    if (ret < 0) {
        dev_err(&client->dev, "error writing to device\n");
        return ret;
    }

    udelay(1000);

    buf[0] = 0x00;
    ret = i2c_master_send(client, buf, 1);
    if (ret < 0) {
        dev_err(&client->dev, "error writing to device\n");
        return ret;
    }

    nunchuck_read_registers(client, recv);

    input = devm_input_allocate_device(&client->dev);
    if (!input)
        return -ENOMEM;

    nunchuk->i2c_client = client;
    input_set_drvdata(input, nunchuk);
    
    input->name = "Wii Nunchuk";
    input->id.bustype = BUS_I2C;

    set_bit(EV_KEY, input->evbit);
    set_bit(BTN_C, input->keybit);
    set_bit(BTN_Z, input->keybit);

    set_bit(EV_ABS, input->evbit);
    set_bit(ABS_X, input->absbit);
    set_bit(ABS_Y, input->absbit);
    input_set_abs_params(input, ABS_X, 30, 220, 4, 8);
    input_set_abs_params(input, ABS_Y, 40, 200, 4, 8);

    /* Classic buttons */
    set_bit(BTN_TL, input->keybit);
    set_bit(BTN_SELECT, input->keybit);
    set_bit(BTN_MODE, input->keybit);
    set_bit(BTN_START, input->keybit);
    set_bit(BTN_TR, input->keybit);
    set_bit(BTN_TL2, input->keybit);
    set_bit(BTN_B, input->keybit);
    set_bit(BTN_Y, input->keybit);
    set_bit(BTN_A, input->keybit);
    set_bit(BTN_X, input->keybit);
    set_bit(BTN_TR2, input->keybit);

    ret = input_setup_polling(input, nunchuk_poll);
    if (ret) {
        dev_err(&client->dev, "failed to set polling function (%d)\n", ret);
        return ret;
    }

    input_set_poll_interval(input, 50);

    ret = input_register_device(input);
    if (ret) {
        dev_err(&client->dev, "cannot register input device (%d)", ret);
        return ret;
    }

    return 0;
}

static void nunchuk_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "nunchuk_remove\n");
}

static struct i2c_driver nunchuk_driver = {
    .driver = {
        .name = "nunchuk",
        .of_match_table = nunchuk_of_match,
    },
    .probe_new = nunchuk_probe,
    .remove = nunchuk_remove
};
module_i2c_driver(nunchuk_driver);

MODULE_LICENSE("GPL");