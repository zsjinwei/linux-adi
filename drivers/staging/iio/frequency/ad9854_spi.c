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

// struct ad9854_reg ad9854_reg
// {
// 	.update_clock_ll = 0x40,
// 	 .control_reg_hm = 0x10,
// 	  .control_reg_lm = 0x64,
// 	   .control_reg_hl = 0x01,
// 	    .control_reg_ll = 0x20,
// 	     .osk_ramp_rate = 0x80,
// };

static int ad9854_io_reset(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_io_reset)) {
		gpio_set_value(st->pdata->gpio_io_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpio_set_value(st->pdata->gpio_io_reset, 0);
		return 0;
	}

	return -ENODEV;
}

static int ad9854_master_reset(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_m_reset)) {
		gpio_set_value(st->pdata->gpio_m_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpio_set_value(st->pdata->gpio_m_reset, 0);
		return 0;
	}

	return -ENODEV;
}

static void ad9854_init(struct ad9854_state *st)
{

}

static int ad9854_request_gpios(struct ad9854_state *st)
{
	int ret;

	if (gpio_is_valid(st->pdata->gpio_osk)) {
		ret = gpio_request_one(st->pdata->gpio_osk,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_OSK");
		if (ret) {
			dev_err(st->dev, "failed to request GPIO OSK\n");
			goto error_ret;
		}
	} else {
		ret = -EIO;
		goto error_ret;
	}

	if (gpio_is_valid(st->pdata->gpio_fsk_bpsk_hold)) {
		ret = gpio_request_one(st->pdata->gpio_fsk_bpsk_hold,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_FSK_BPSK_HOLD");
		if (ret) {
			dev_err(st->dev, "failed to request GPIO FSK/BPSK/HOLD\n");
			goto error_free_osk;
		}
	} else {
		ret = -EIO;
		goto error_free_osk;
	}

	if (gpio_is_valid(st->pdata->gpio_io_ud_clk)) {
		ret = gpio_request_one(st->pdata->gpio_io_ud_clk,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_IO_UD_CLK");
		if (ret) {
			dev_err(st->dev, "failed to request GPIO I/O UD CLK\n");
			goto error_free_fskbpskhold;
		}
	} else {
		ret = -EIO;
		goto error_free_fskbpskhold;
	}

	if (gpio_is_valid(st->pdata->gpio_m_reset)) {
		ret = gpio_request_one(st->pdata->gpio_m_reset,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_MASTER_RESET");
		if (ret) {
			dev_err(st->dev, "failed to request GPIO MASTER RESET\n");
			goto error_free_ioudclk;
		}
	} else {
		ret = -EIO;
		goto error_free_ioudclk;
	}

	if (gpio_is_valid(st->pdata->gpio_io_reset)) {
		ret = gpio_request_one(st->pdata->gpio_io_reset,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_IO_RESET");
		if (ret) {
			dev_err(st->dev, "failed to request GPIO IO RESET\n");
			goto error_free_mreset;
		}
	} else {
		ret = -EIO;
		goto error_free_mreset;
	}

	if (gpio_is_valid(st->pdata->gpio_sp_select)) {
		ret = gpio_request_one(st->pdata->gpio_sp_select,
		                       GPIOF_OUT_INIT_LOW,
		                       "AD9854_SP_SELECT");
		if (ret) {
			dev_err(st->dev, "failed to request GPIO S/P SELECT\n");
			goto error_free_ioreset;
		}
	} else {
		ret = -EIO;
		goto error_free_ioreset;
	}

	return 0;

error_free_ioreset:
	if (gpio_is_valid(st->pdata->gpio_io_reset))
		gpio_free(st->pdata->gpio_io_reset);
error_free_mreset:
	if (gpio_is_valid(st->pdata->gpio_m_reset))
		gpio_free(st->pdata->gpio_m_reset);
error_free_ioudclk:
	if (gpio_is_valid(st->pdata->gpio_io_ud_clk))
		gpio_free(st->pdata->gpio_io_ud_clk);
error_free_fskbpskhold:
	if (gpio_is_valid(st->pdata->gpio_fsk_bpsk_hold))
		gpio_free(st->pdata->gpio_fsk_bpsk_hold);
error_free_osk:
	if (gpio_is_valid(st->pdata->gpio_osk))
		gpio_free(st->pdata->gpio_osk);
error_ret:
	return ret;
}

static void ad7606_free_gpios(struct ad9854_state *st)
{
	if (gpio_is_valid(st->pdata->gpio_sp_select))
		gpio_free(st->pdata->gpio_sp_select);
	if (gpio_is_valid(st->pdata->gpio_io_reset))
		gpio_free(st->pdata->gpio_io_reset);
	if (gpio_is_valid(st->pdata->gpio_m_reset))
		gpio_free(st->pdata->gpio_m_reset);
	if (gpio_is_valid(st->pdata->gpio_io_ud_clk))
		gpio_free(st->pdata->gpio_io_ud_clk);
	if (gpio_is_valid(st->pdata->gpio_fsk_bpsk_hold))
		gpio_free(st->pdata->gpio_fsk_bpsk_hold);
	if (gpio_is_valid(st->pdata->gpio_osk))
		gpio_free(st->pdata->gpio_osk);
}

/**
 * [ad9854_spi_read_reg read value
 * from device to ad9854_ser_reg struct]
 * @param  dev spi device
 * @param  register which read from
 * @return     0 - no error
 */
static int ad9854_spi_read_reg(struct device *dev,
                               struct ad9854_ser_reg *reg)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad9854_state *st = iio_priv(indio_dev);
	struct spi_device *spi = to_spi_device(dev);
	int i, ret;
	unsigned long long reg_val = 0;
	char *data = (char *) &reg_val;
	char *data_tmp = data + (6 - reg->reg_len);
	char tx_inst = AD9854_INST_ADDR_R(reg->reg_addr);

	// I/O reset before operation in order to synchronization with the AD9854
	ad9854_io_reset(st);
	// write instruction and read
	ret = spi_write_then_read(spi, &tx_inst, 1, data_tmp, reg->reg_len);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI read error\n");
		return ret;
	}

	reg->reg_val = be64_to_cpu(reg_val);

	return 0;
}

/**
 * [ad9854_spi_write_reg write value
 * from ad9854_ser_reg struct to device]
 * @param  dev spi device
 * @param  reg register which write to
 * @return     0 - no error
 */
static int ad9854_spi_write_reg(struct device *dev,
                                struct ad9854_ser_reg *reg)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad9854_state *st = iio_priv(indio_dev);
	struct spi_device *spi = to_spi_device(dev);
	int i, ret;
	unsigned long long reg_val = cpu_to_be64(reg->reg_val);  // use MSB as default
	char *data = (char *) &reg_val;
	data += (6 - reg->reg_len);
	char tx_inst = AD9854_INST_ADDR_W(reg->reg_addr);

	// I/O reset before operation in order to synchronization with the AD9854
	ad9854_io_reset(st);
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
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9854_ser_reg *reg;
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32) this_attr->address) {
	case AD9854_REG_SER_PHASE_ADJ_1:
	case AD9854_REG_SER_PHASE_ADJ_2:
	case AD9854_REG_SER_FREQ_TUNING_WORD_1:
	case AD9854_REG_SER_FREQ_TUNING_WORD_2:
	case AD9854_REG_SER_DELTA_FREQ_WORD:
	case AD9854_REG_SER_UPDATE_CLOCK:
	case AD9854_REG_SER_RAMP_RATE_CLOCK:

	case AD9854_REG_SER_OUTPUT_I_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_Q_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_RAMP_RATE:
	case AD9854_REG_SER_QDAC:
		reg = &ad9854_ser_reg_tbl[this_attr->address];
		ret = ad9854_spi_read_reg(dev, reg);
		break;
	case AD9854_REG_SER_CTRL:
	case AD9854_PHASE_SYM:
	case AD9854_FREQ_SYM:
	case AD9854_PINCTRL_EN:
	case AD9854_OUTPUT_EN:
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return sprintf(buf, "%u\n", st->oversampling);
}

static ssize_t ad9854_write(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad9854_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9854_ser_reg *reg;
	int ret;
	unsigned long long val, old_val;

	ret = kstrtoull(buf, 15, &val);
	if (ret)
		goto error_ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32) this_attr->address) {
	case AD9854_REG_SER_PHASE_ADJ_1:
	case AD9854_REG_SER_PHASE_ADJ_2:
	case AD9854_REG_SER_FREQ_TUNING_WORD_1:
	case AD9854_REG_SER_FREQ_TUNING_WORD_2:
	case AD9854_REG_SER_DELTA_FREQ_WORD:
	case AD9854_REG_SER_UPDATE_CLOCK:
	case AD9854_REG_SER_RAMP_RATE_CLOCK:

	case AD9854_REG_SER_OUTPUT_I_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_Q_MULTIPLIER:
	case AD9854_REG_SER_OUTPUT_RAMP_RATE:
	case AD9854_REG_SER_QDAC:
		reg = &ad9854_ser_reg_tbl[this_attr->address];
		old_val = reg->reg_val;
		reg->reg_val = val;
		ret = ad9854_spi_write_reg(dev, reg);
		if (ret)
			reg->reg_val = old_val;  // write error, restore old value.
		break;
	case AD9854_REG_SER_CTRL:
	case AD9854_PHASE_SYM:
	case AD9854_FREQ_SYM:
	case AD9854_PINCTRL_EN:
	case AD9854_OUTPUT_EN:
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

error_ret:
	return ret ? ret : len;
}

/**
 * see dds.h for further information
 */

static IIO_DEV_ATTR_FREQ(0, 0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_FREQ_TUNING_WORD_1);
static IIO_DEV_ATTR_FREQ(0, 1, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_FREQ_TUNING_WORD_2);
static IIO_DEV_ATTR_FREQSYMBOL(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_FREQ_SYM);
static IIO_CONST_ATTR_FREQ_SCALE(0, "1"); /* 1Hz */

static IIO_DEV_ATTR_PHASE(0, 0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_PHASE_ADJ_1);
static IIO_DEV_ATTR_PHASE(0, 1, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_PHASE_ADJ_2);
static IIO_DEV_ATTR_PHASESYMBOL(0, S_IWUSR, ad9854_read, ad9854_write, ad9854_PHASE_SYM);
static IIO_CONST_ATTR_PHASE_SCALE(0, "0.0015339808"); /* 2PI/2^12 rad*/

static IIO_DEV_ATTR_PINCONTROL_EN(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_PINCTRL_EN);
static IIO_DEV_ATTR_OUT_ENABLE(0, S_IWUSR, ad9854_read, ad9854_write, ad9854_OUTPUT_EN);

static IIO_DEV_ATTR_DELTAFREQ(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_DELTA_FREQ_WORD);
static IIO_DEV_ATTR_UPDATECLK(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_UPDATE_CLOCK);
static IIO_DEV_ATTR_RAMPRATECLK(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_RAMP_RATE_CLOCK);
static IIO_DEV_ATTR_OSK_IMULTI(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_OUTPUT_I_MULTIPLIER);
static IIO_DEV_ATTR_OSK_QMULTI(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_OUTPUT_Q_MULTIPLIER);
static IIO_DEV_ATTR_OSK_RAMPRATE(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_OUTPUT_RAMP_RATE);
static IIO_DEV_ATTR_QDAC(0, S_IWUSR, ad9854_read, ad9854_write, AD9854_REG_SER_QDAC);

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
	unsigned gpio_osk;
	unsigned gpio_fsk_bpsk_hold;
	unsigned gpio_io_ud_clk;
	unsigned gpio_m_reset;
	unsigned gpio_io_reset;
	unsigned gpio_sp_select;
	pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
		return NULL; /* out of memory */

	/* no such property */
	if (of_property_read_u32(node, "ad9854,default_os", &pdata->default_os) != 0)
	{
		dev_err(&spi->dev, "default_os property is not defined.\n");
		return NULL;
	}

	/* now get the gpio number*/
	gpio_osk = of_get_named_gpio(node, "ad9854,gpio_osk", 0);
	if (IS_ERR_VALUE(gpio_osk)) {
		dev_warn(&spi->dev, "gpio_osk can not setup, set it to -1.\n");
		pdata->gpio_osk = -1;
	}
	else
	{
		pdata->gpio_osk = gpio_osk;
	}
	/* now get the gpio number*/
	gpio_fsk_bpsk_hold = of_get_named_gpio(node, "ad9854,gpio_fsk_bpsk_hold", 0);
	if (IS_ERR_VALUE(gpio_fsk_bpsk_hold)) {
		dev_warn(&spi->dev, "gpio_fsk_bpsk_hold can not setup, set it to -1.\n");
		pdata->gpio_fsk_bpsk_hold = -1;
	}
	else
	{
		pdata->gpio_fsk_bpsk_hold = gpio_fsk_bpsk_hold;
	}
	/* now get the gpio number*/
	gpio_io_ud_clk = of_get_named_gpio(node, "ad9854,gpio_io_ud_clk", 0);
	if (IS_ERR_VALUE(gpio_io_ud_clk)) {
		dev_warn(&spi->dev, "gpio_io_ud_clk can not setup, set it to -1.\n");
		pdata->gpio_io_ud_clk = -1;
	}
	else
	{
		pdata->gpio_io_ud_clk = gpio_io_ud_clk;
	}
	/* now get the gpio number*/
	gpio_m_reset = of_get_named_gpio(node, "ad9854,gpio_m_reset", 0);
	if (IS_ERR_VALUE(gpio_m_reset)) {
		dev_warn(&spi->dev, "gpio_m_reset can not setup, set it to -1.\n");
		pdata->gpio_m_reset = -1;
	}
	else
	{
		pdata->gpio_m_reset = gpio_m_reset;
	}
	/* now get the gpio number*/
	gpio_io_reset = of_get_named_gpio(node, "ad9854,gpio_io_reset", 0);
	if (IS_ERR_VALUE(gpio_io_reset)) {
		dev_warn(&spi->dev, "gpio_io_reset can not setup, set it to -1.\n");
		pdata->gpio_io_reset = -1;
	}
	else
	{
		pdata->gpio_io_reset = gpio_io_reset;
	}
	/* now get the gpio number*/
	gpio_sp_select = of_get_named_gpio(node, "ad9854,gpio_sp_select", 0);
	if (IS_ERR_VALUE(gpio_sp_select)) {
		dev_warn(&spi->dev, "gpio_sp_select can not setup, set it to -1.\n");
		pdata->gpio_sp_select = -1;
	}
	else
	{
		pdata->gpio_sp_select = gpio_sp_select;
	}

	dev_info(&spi->dev, "DT parse result:\ngpio_osk = %d.\ngpio_fsk_bpsk_hold = %d.\ngpio_io_ud_clk = %d.\ngpio_m_reset = %d.\ngpio_io_reset = %d.\ngpio_sp_select = %d.\n",
	         pdata->gpio_osk,
	         pdata->gpio_fsk_bpsk_hold,
	         pdata->gpio_io_ud_clk,
	         pdata->gpio_m_reset,
	         pdata->gpio_io_reset,
	         pdata->gpio_sp_select);
	/* pdata is filled */
	return pdata;
}
//=============================================================

static int ad9854_spi_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9854_platform_data *pdata;

	if (spi->dev.of_node != NULL)
	{
		pdata = ad9854_parse_dt(spi);
		if (pdata != NULL)
			spi->dev.platform_data = pdata;
	}

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
