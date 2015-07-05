/*
 * AD7606 SPI ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include "ad9854.h"

#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#define MAX_SPI_FREQ_HZ		10000000

static void ad9854_io_reset(void)
{

}

static void ad9854_master_reset(void)
{

}

static void ad9854_init(struct ad9854_state *st)
{

}

static int ad9854_request_gpios(struct ad9854_state *st)
{

}

static int ad9854_spi_read_reg(struct device *dev,
                               struct ad9854_ser_reg *reg)
{
	struct spi_device *spi = to_spi_device(dev);
	int i, ret;
	char *data = &(reg->reg_val);
	char tx_inst = AD9854_INST_ADDR_R(reg->reg_addr);

	// I/O reset before operation in order to synchronization with the AD9854
	ad9854_io_reset();
	// write instruction and read
	ret = spi_write_then_read(spi, &tx_inst, 1, data, reg->reg_len);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI read error\n");
		return ret;
	}

	return 0;
}

static int ad9854_spi_write_reg(struct device *dev,
                                struct ad9854_ser_reg *reg)
{
	struct spi_device *spi = to_spi_device(dev);
	int i, ret;
	char *data = &(reg->reg_val);
	char tx_inst = AD9854_INST_ADDR_W(reg->reg_addr);

	// I/O reset before operation in order to synchronization with the AD9854
	ad9854_io_reset();
	// write instruction
	ret = spi_write(spi, &tx_inst, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI write instruction error\n");
		return ret;
	}

	ret = spi_write(spi, data, reg->reg_len);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI write data error\n");
		return ret;
	}

	return 0;
}

static const struct ad9854_bus_ops ad9854_spi_bops = {
	.read_block	= ad9854_spi_read_block,
};

static ssize_t ad9854_read(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%u\n", st->oversampling);
}

static ssize_t ad9854_write(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t len)
{

}

/**
 * see dds.h for further information
 */

static IIO_DEV_ATTR_FREQ(0, 0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_FREQ_TUNING_WORD_1);
static IIO_DEV_ATTR_FREQ(0, 1, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_FREQ_TUNING_WORD_2);
static IIO_DEV_ATTR_FREQSYMBOL(0, S_IWUSR, ad9854_read, ad9854_write, ad9854_FREQ_SYM);
static IIO_CONST_ATTR_FREQ_SCALE(0, "1"); /* 1Hz */

static IIO_DEV_ATTR_PHASE(0, 0, S_IWUSR, ad9854_read, ad9854_write, ad9854_PHASE0H);
static IIO_DEV_ATTR_PHASE(0, 1, S_IWUSR, ad9854_read, ad9854_write, ad9854_PHASE1H);
static IIO_DEV_ATTR_PHASESYMBOL(0, S_IWUSR, ad9854_read, ad9854_write, ad9854_PHASE_SYM);
static IIO_CONST_ATTR_PHASE_SCALE(0, "0.0015339808"); /* 2PI/2^12 rad*/

static IIO_DEV_ATTR_PINCONTROL_EN(0, S_IWUSR, ad9854_read, ad9854_write, ad9854_PINCTRL_EN);
static IIO_DEV_ATTR_OUT_ENABLE(0, S_IWUSR, ad9854_read, ad9854_write, ad9854_OUTPUT_EN);

static IIO_DEV_ATTR_DELTAFREQ(0, S_IWUSR, ad9854_read, ad9854_write, _addr);
static IIO_DEV_ATTR_UPDATECLK(0, S_IWUSR, ad9854_read, ad9854_write, _addr);
static IIO_DEV_ATTR_RAMPRATECLK(0, S_IWUSR, ad9854_read, ad9854_write, _addr);
static IIO_DEV_ATTR_OSK_IMULTI(0, S_IWUSR, ad9854_read, ad9854_write, _addr);
static IIO_DEV_ATTR_OSK_IMULTI(0, S_IWUSR, ad9854_read, ad9854_write, _addr);
static IIO_DEV_ATTR_OSK_IMULTI(0, S_IWUSR, ad9854_read, ad9854_write, _addr);
static IIO_DEV_ATTR_OSK_IMULTI(0, S_IWUSR, ad9854_read, ad9854_write, _addr);

static struct attribute *ad9854_attributes[] = {
	&iio_dev_attr_out_altvoltage0_frequency0.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_frequency1.dev_attr.attr,
	&iio_const_attr_out_altvoltage0_frequency_scale.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_phase0.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_phase1.dev_attr.attr,
	&iio_const_attr_out_altvoltage0_phase_scale.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_pincontrol_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_frequencysymbol.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_phasesymbol.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_out_enable.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9854_attribute_group = {
	.attrs = ad9854_attributes,
};

static const struct iio_info ad9854_info = {
	.attrs = &ad9854_attribute_group,
	.driver_module = THIS_MODULE,
};

static struct ad9854_platform_data *
ad9854_parse_dt(struct spi_device *spi)
{
	struct device_node *node = spi->dev.of_node;
	struct ad9854_platform_data *pdata;
	unsigned gpio_convst, gpio_reset, gpio_range, gpio_os0, gpio_os1, gpio_os2, gpio_frstdata, gpio_stby;
	pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
		return NULL; /* out of memory */

	/* no such property */
	if (of_property_read_u32(node, "ad9854,default_os", &pdata->default_os) != 0)
	{
		dev_err(&spi->dev, "default_os property is not defined.\n");
		return NULL;
	}
	/* no such property */
	if (of_property_read_u32(node, "ad9854,default_range", &pdata->default_range) != 0)
	{
		dev_err(&spi->dev, "default_range property is not defined.\n");
		return NULL;
	}

	/* now get the gpio number*/
	gpio_convst = of_get_named_gpio(node, "ad9854,gpio_convst", 0);
	if (IS_ERR_VALUE(gpio_convst)) {
		dev_warn(&spi->dev, "gpio_convst can not setup, set it to -1.\n");
		pdata->gpio_convst = -1;
	}
	else
	{
		pdata->gpio_convst = gpio_convst;
	}

	gpio_reset = of_get_named_gpio(node, "ad9854,gpio_reset", 0);
	if (IS_ERR_VALUE(gpio_reset)) {
		dev_warn(&spi->dev, "gpio_reset can not setup, set it to -1.\n");
		pdata->gpio_reset = -1;
	}
	else
	{
		pdata->gpio_reset = gpio_reset;
	}

	gpio_range = of_get_named_gpio(node, "ad9854,gpio_range", 0);
	if (IS_ERR_VALUE(gpio_range)) {
		dev_warn(&spi->dev, "gpio_range can not setup, set it to -1.\n");
		pdata->gpio_range = -1;
	}
	else
	{
		pdata->gpio_range = gpio_range;
	}

	gpio_os0 = of_get_named_gpio(node, "ad9854,gpio_os0", 0);
	if (IS_ERR_VALUE(gpio_os0)) {
		dev_warn(&spi->dev, "gpio_os0 can not setup, set it to -1.\n");
		pdata->gpio_os0 = -1;
	}
	else
	{
		pdata->gpio_os0 = gpio_os0;
	}

	gpio_os1 = of_get_named_gpio(node, "ad9854,gpio_os1", 0);
	if (IS_ERR_VALUE(gpio_os1)) {
		dev_warn(&spi->dev, "gpio_os1 can not setup, set it to -1.\n");
		pdata->gpio_os1 = -1;
	}
	else
	{
		pdata->gpio_os1 = gpio_os1;
	}

	gpio_os2 = of_get_named_gpio(node, "ad9854,gpio_os2", 0);
	if (IS_ERR_VALUE(gpio_os2)) {
		dev_warn(&spi->dev, "gpio_os2 can not setup, set it to -1.\n");
		pdata->gpio_os2 = -1;
	}
	else
	{
		pdata->gpio_os2 = gpio_os2;
	}

	gpio_frstdata = of_get_named_gpio(node, "ad9854,gpio_frstdata", 0);
	if (IS_ERR_VALUE(gpio_frstdata)) {
		dev_warn(&spi->dev, "gpio_frstdata can not setup, set it to -1.\n");
		pdata->gpio_frstdata = -1;
	}
	else
	{
		pdata->gpio_frstdata = gpio_frstdata;
	}

	gpio_stby = of_get_named_gpio(node, "ad9854,gpio_stby", 0);
	if (IS_ERR_VALUE(gpio_stby)) {
		dev_warn(&spi->dev, "gpio_stby can not setup, set it to -1.\n");
		pdata->gpio_stby = -1;
	}
	else
	{
		pdata->gpio_stby = gpio_stby;
	}
	dev_info(&spi->dev, "DT parse result:\ndefault_os=%d.\ndefault_range=%d.\ngpio_convst=%d.\ngpio_reset=%d.\ngpio_range=%d.\ngpio_os0=%d.\ngpio_os1=%d.\ngpio_os2=%d.\ngpio_frstdata=%d.\ngpio_stby=%d.\n",
	         pdata->default_os, pdata->default_range, pdata->gpio_convst,
	         pdata->gpio_reset, pdata->gpio_range, pdata->gpio_os0, pdata->gpio_os1,
	         pdata->gpio_os2, pdata->gpio_frstdata, pdata->gpio_stby);
	/* pdata is filled */
	return pdata;
}
//=============================================================

static int ad9854_spi_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
//=============== add by HJW 2015-06-17 =========================
	struct ad9854_platform_data *pdata;

	if (spi->dev.of_node != NULL)
	{
		pdata = ad9854_parse_dt(spi);
		if (pdata != NULL)
			spi->dev.platform_data = pdata;
	}
//=============================================================
	indio_dev = ad9854_probe(&spi->dev, spi->irq, NULL,
	                         spi_get_device_id(spi)->driver_data,
	                         &ad9854_spi_bops);

	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	spi_set_drvdata(spi, indio_dev);

	return 0;
}

static int ad9854_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&spi->dev);

	return ad9854_remove(indio_dev, spi->irq);
}

#ifdef CONFIG_PM
static int ad9854_spi_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	ad9854_suspend(indio_dev);

	return 0;
}

static int ad9854_spi_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	ad9854_resume(indio_dev);

	return 0;
}

static const struct dev_pm_ops ad9854_pm_ops = {
	.suspend = ad9854_spi_suspend,
	.resume  = ad9854_spi_resume,
};
#define AD7606_SPI_PM_OPS (&ad9854_pm_ops)

#else
#define AD7606_SPI_PM_OPS NULL
#endif

static const struct spi_device_id ad9854_id[] = {
	{"ad9854", ID_AD9854},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9854_id);

#ifdef CONFIG_OF
static const struct of_device_id ad9854_dt_ids[] = {
	{ .compatible = "adi,ad9854" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ad9854_dt_ids);
#endif

static struct spi_driver ad9854_driver = {
	.driver = {
		.name = "ad9854",
		.owner = THIS_MODULE,
		.pm    = AD7606_SPI_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ad9854_dt_ids),
#endif
	},
	.probe = ad9854_spi_probe,
	.remove = ad9854_spi_remove,
	.id_table = ad9854_id,
};
module_spi_driver(ad9854_driver);

MODULE_AUTHOR("JinWei Hwang <zsjinwei@live.com>");
MODULE_DESCRIPTION("Analog Devices AD9854 Driver");
MODULE_LICENSE("GPL v2");
