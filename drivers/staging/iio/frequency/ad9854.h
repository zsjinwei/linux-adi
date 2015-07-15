/*
 * AD9854 SPI DDS driver
 *
 * Copyright 2015 SYSU SIST JinWei Hwang.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_DDS_AD9854_H_
#define IIO_DDS_AD9854_H_

/* Registers */
#define AD9854_REG_SER_SIZE		12
#define AD9854_REG_SER_CTRL_DEFAULT_VAL		0x10640120

#define AD9854_REG_SER_PHASE_ADJ_1		0x00
#define AD9854_REG_SER_PHASE_ADJ_2		0x01
#define AD9854_REG_SER_FREQ_TUNING_WORD_1		0x02
#define AD9854_REG_SER_FREQ_TUNING_WORD_2		0x03
#define AD9854_REG_SER_DELTA_FREQ_WORD		0x04
#define AD9854_REG_SER_UPDATE_CLOCK		0x05
#define AD9854_REG_SER_RAMP_RATE_CLOCK		0x06
#define AD9854_REG_SER_CTRL		0x07
#define AD9854_REG_SER_OUTPUT_I_MULTIPLIER		0x08
#define AD9854_REG_SER_OUTPUT_Q_MULTIPLIER		0x09
#define AD9854_REG_SER_OUTPUT_RAMP_RATE		0x0A
#define AD9854_REG_SER_QDAC		0x0B

#define AD9854_ATTR_PHASE_SYM		0x30
#define AD9854_ATTR_FREQ_SYM		0x31
#define AD9854_ATTR_PINCTRL_EN		0x32
#define AD9854_ATTR_OUTPUT_EN		0x33

#define AD9854_ATTR_OSK_EN		0x34
#define AD9854_ATTR_INVSINC_EN		0x35
#define AD9854_ATTR_MODES		0x36
#define AD9854_ATTR_RESET		0x37
#define AD9854_ATTR_QDAC_PD		0x38
#define AD9854_ATTR_DAC_PD		0x39

/* Instruction byte */
#define AD9854_INST_R		(1<<7)
#define AD9854_INST_W		(0<<7)
#define AD9854_INST_ADDR_R(addr)		(AD9854_INST_R | (addr & (0x0F)))
#define AD9854_INST_ADDR_W(addr)		(AD9854_INST_W | (addr & (0x0F)))

/* control reg bits(31:24) */
#define CTRL_CR_COMP_PD		(1<<28)
#define CTRL_CR_QDAC_PD		(1<<26)
#define CTRL_CR_DAC_PD		(1<<25)
#define CTRL_CR_DIG_PD		(1<<24)
/* control reg bits(24:16) */
#define CTRL_CR_PLL_RANGE		(1<<22)
#define CTRL_CR_BYPASS_PLL		(1<<21)
#define CTRL_CR_REF_MULT_4		(1<<20)
#define CTRL_CR_REF_MULT_3		(1<<19)
#define CTRL_CR_REF_MULT_2		(1<<18)
#define CTRL_CR_REF_MULT_1		(1<<17)
#define CTRL_CR_REF_MULT_0		(1<<16)
/* control reg bits(15:8) */
#define CTRL_CR_CLR_ACC_1		(1<<15)
#define CTRL_CR_CLR_ACC_2		(1<<14)
#define CTRL_CR_TRIANGLE		(1<<13)
#define CTRL_CR_SRC_QDAC		(1<<12)
#define CTRL_CR_MODE_2		(1<<11)
#define CTRL_CR_MODE_1		(1<<10)
#define CTRL_CR_MODE_0		(1<<9)
#define CTRL_CR_IN_EXT_UP_CLK		(1<<8)
/* control reg bits(7:0) */
#define CTRL_CR_BYPASS_INV_SINC		(1<<6)
#define CTRL_CR_OSK_EN		(1<<5)
#define CTRL_CR_OSK_INT		(1<<4)
#define CTRL_CR_LSB_FIRST		(1<<1)
#define CTRL_CR_SDO_ACTIVE		(1<<0)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_deltafrequency (just for ad9854)
 */

#define IIO_DEV_ATTR_DELTAFREQ(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_deltafrequency,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_updateclk (just for ad9854)
 */

#define IIO_DEV_ATTR_UPDATECLK(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_updateclk,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ramprateclk (just for ad9854)
 */

#define IIO_DEV_ATTR_RAMPRATECLK(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ramprateclk,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_imulti (just for ad9854)
 */

#define IIO_DEV_ATTR_OSK_IMULTI(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_imulti,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_qmulti (just for ad9854)
 */

#define IIO_DEV_ATTR_OSK_QMULTI(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_qmulti,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_ramprate (just for ad9854)
 */

#define IIO_DEV_ATTR_OSK_RAMPRATE(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_ramprate,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_qdac (just for ad9854)
 */

#define IIO_DEV_ATTR_QDAC(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_qdac,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_qdac (just for ad9854)
 */

#define IIO_DEV_ATTR_QDAC_PD(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_qdac_pd,	\
			_mode, _show, _store, _addr)

/**
* /sys/bus/iio/devices/.../out_altvoltageX_qdac (just for ad9854)
*/

#define IIO_DEV_ATTR_DAC_PD(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_dac_pd,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_osk_en (just for ad9854)
 */

#define IIO_DEV_ATTR_OSK_EN(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_osk_en,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_invsinc_en (just for ad9854)
 */

#define IIO_DEV_ATTR_INVSINC_EN(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_invsinc_en,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_modes (just for ad9854)
 */

#define IIO_DEV_ATTR_MODES(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_modes,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_reset (just for ad9854)
 */

#define IIO_DEV_ATTR_RESET(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_reset,	\
			_mode, _show, _store, _addr)

/**
 * /sys/bus/iio/devices/.../out_altvoltageX_ctrlreg (just for ad9854)
 */

#define IIO_DEV_ATTR_CTRLREG(_channel, _mode, _show, _store, _addr)	\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_ctrlreg,	\
			_mode, _show, _store, _addr)

//struct ad9854_reg {
//	char phase_adjust_reg_1_hl;		/* ad9854 Phase Adjust Register 1 <13:8> */
//	char phase_adjust_reg_1_ll;		/* ad9854 Phase Adjust Register 1 <7:0> */
//
//	char phase_adjust_reg_2_hl;		/* ad9854 Phase Adjust Register 2 <13:8> */
//	char phase_adjust_reg_2_ll;		/* ad9854 Phase Adjust Register 2 <7:0> */
//
//	char freq_tuning_word_1_hh;		/* ad9854 Frequency Tuning Word 1 <47:40> */
//	char freq_tuning_word_1_lh;		/* ad9854 Frequency Tuning Word 1 <39:32> */
//	char freq_tuning_word_1_hm;		/* ad9854 Frequency Tuning Word 1 <31:24> */
//	char freq_tuning_word_1_lm;		/* ad9854 Frequency Tuning Word 1 <23:16> */
//	char freq_tuning_word_1_hl;		/* ad9854 Frequency Tuning Word 1 <15:8> */
//	char freq_tuning_word_1_ll;		/* ad9854 Frequency Tuning Word 1 <7:0> */
//
//	char freq_tuning_word_2_hh;		/* ad9854 Frequency Tuning Word 2 <47:40> */
//	char freq_tuning_word_2_lh;		/* ad9854 Frequency Tuning Word 2 <39:32> */
//	char freq_tuning_word_2_hm;		/* ad9854 Frequency Tuning Word 2 <31:24> */
//	char freq_tuning_word_2_lm;		/* ad9854 Frequency Tuning Word 2 <23:16> */
//	char freq_tuning_word_2_hl;		/* ad9854 Frequency Tuning Word 2 <15:8> */
//	char freq_tuning_word_2_ll;		/* ad9854 Frequency Tuning Word 2 <7:0> */
//
//	char delta_freq_word_hh;		/* ad9854 Delta requency word <47:40> */
//	char delta_freq_word_lh;		/* ad9854 Delta requency word <39:32> */
//	char delta_freq_word_hm;		/* ad9854 Delta requency word <31:24> */
//	char delta_freq_word_lm;		/* ad9854 Delta requency word <23:16> */
//	char delta_freq_word_hl;		/* ad9854 Delta requency word <15:8> */
//	char delta_freq_word_ll;		/* ad9854 Delta requency word <7:0> */
//
//	char update_clock_hm;		/* ad9854 Update clock <31:24> */
//	char update_clock_lm;		/* ad9854 Update clock <23:16> */
//	char update_clock_hl;		/* ad9854 Update clock <15:8> */
//	char update_clock_ll;		/* ad9854 Update clock <7:0> */
//
//	char ramp_rate_clock_lm;		/* ad9854 Ramp rate clock <19:16> */
//	char ramp_rate_clock_hl;		/* ad9854 Ramp rate clock <15:8> */
//	char ramp_rate_clock_ll;		/* ad9854 Ramp rate clock <7:0> */
//
//	char control_reg_hm;		/* ad9854 Control register <31:24> */
//	char control_reg_lm;		/* ad9854 Control register <23:16> */
//	char control_reg_hl;		/* ad9854 Control register <15:8> */
//	char control_reg_ll;		/* ad9854 Control register <7:0> */
//
//	char osk_i_multiplier_hl;		/* ad9854 Output shaped keying I multiplier <11:8> */
//	char osk_i_multiplier_ll;		/* ad9854 Output shaped keying I multiplier <7:0> */
//
//	char osk_q_multiplier_hl;		/* ad9854 Output shaped keying Q multiplier <11:8> */
//	char osk_q_multiplier_ll;		/* ad9854 Output shaped keying Q multiplier <7:0> */
//
//	char osk_ramp_rate;		/* ad9854 Output shaped keying ramp rate <7:0> */
//
//	char qdac_hl;		/* ad9854 QDAC <11:8> */
//	char qdac_ll;		/* ad9854 QDAC <7:0> */
//};

/**
 * struct ad9854_platform_data - platform specific information
 * @mclk:		master clock in Hz
 * @freq0:		power up freq0 tuning word in Hz
 * @freq1:		power up freq1 tuning word in Hz
 * @phase0:		power up phase0 value [0..4095] correlates with 0..2PI
 * @phase1:		power up phase1 value [0..4095] correlates with 0..2PI
 * @en_div2:		digital output/2 is passed to the SIGN BIT OUT pin
 * @en_signbit_msb_out:	the MSB (or MSB/2) of the DAC data is connected to the
 *			SIGN BIT OUT pin. en_div2 controls whether it is the MSB
 *			or MSB/2 that is output. if en_signbit_msb_out=false,
 *			the on-board comparator is connected to SIGN BIT OUT
 */

struct ad9854_platform_data {
	unsigned int		ref_clk;
	unsigned int		ref_mult;
	bool		en_lsb_first;
	bool		en_pll_bypass;
	unsigned		gpio_osk;
	unsigned		gpio_fsk_bpsk_hold;
	unsigned		gpio_io_ud_clk;
	unsigned		gpio_m_reset;
	unsigned		gpio_io_reset;
	unsigned		gpio_sp_select;
};

/**
 * struct ad9854_state - driver instance specific data
 * @spi:		spi_device
 * @reg:		supply regulator
 * @mclk:		external master clock
 * @control:		cached control word
 * @xfer:		default spi transfer
 * @msg:		default spi message
 * @freq_xfer:		tuning word spi transfer
 * @freq_msg:		tuning word spi message
 * @data:		spi transmit buffer
 * @freq_data:		tuning word spi transmit buffer
 */

struct ad9854_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	struct ad9854_platform_data	*pdata;
	const struct ad9854_bus_ops	*bops;
	struct ad9854_ser_reg *ser_regs;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16				data ____cacheline_aligned;
};

struct ad9854_ser_reg
{
	unsigned int reg_addr;
	unsigned int reg_len;
	u64 reg_val;
};

struct ad9854_bus_ops {
	/* more methods added in future? */
	int (*read_reg)(struct ad9854_state *st, unsigned int reg_addr);
	int (*write_reg)(struct ad9854_state *st, unsigned int reg_addr);
};

/**
 * ad9854_supported_device_ids:
 */

enum ad9854_supported_device_ids {
	ID_AD9854,
};

static int ad9854_io_reset(struct ad9854_state *st);
static int ad9854_master_reset(struct ad9854_state *st);
static int ad9854_io_update(struct ad9854_state *st);
static int ad9854_spi_read_reg(struct ad9854_state *st, unsigned int reg_addr);
static int ad9854_spi_write_reg(struct ad9854_state *st, unsigned int reg_addr);
static int ad9854_ctrl_reg_init(struct ad9854_state *st);
static int ad9854_gpio_init(struct ad9854_state *st, int gpio);
static int ad9854_check_ser_reg_val(unsigned int reg_addr, u64 reg_val);

#endif /* IIO_DDS_AD9854_H_ */
