/*
 * Copyright (c) 2023 Grinn
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad594x

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/mfd/ad5594x.h>

#define AD594x_GPIO_READBACK_EN BIT(10)
#define AD594x_LDAC_READBACK_EN BIT(6)
#define AD594x_REG_SOFTWARE_RESET 0x0FU
#define AD594x_SOFTWARE_RESET_MAGIC_VAL 0x5AC
#define AD594x_REG_VAL_MASK 0x3FF
#define AD594x_REG_RESET_VAL_MASK 0x7FF
#define AD594x_REG_SHIFT_VAL 11
#define AD594x_REG_READBACK_SHIFT_VAL 2

#define AD594x_SPI_SPEC_CONF (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | \
			      SPI_OP_MODE_MASTER | SPI_MODE_CPOL)

struct mfd_ad594x_config {
	struct gpio_dt_spec reset_gpio;
	struct spi_dt_spec bus;
};

ad594x_err_t mfd_ad594x_read_raw16(const struct device *dev, uint16_t *val)
{
	const struct mfd_ad594x_config *config = dev->config;
	uint16_t nop_msg = 0;

	struct spi_buf tx_buf[] = {
		{
			.buf = &nop_msg,
			.len = sizeof(nop_msg)
		}
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf[] = {
		{
			.buf = val,
			.len = sizeof(uint16_t)
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 1
	};

	return spi_transceive_dt(&config->bus, &tx, &rx);
}

ad594x_err_t mfd_ad594x_read_raw(const struct device *dev, uint8_t *val, uint32_t len)
{
	const struct mfd_ad594x_config *config = dev->config;

	struct spi_buf rx_buf[] = {
		{
			.buf = val,
			.len = len
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 1
	};

	return spi_read_dt(&config->bus, &rx);
}

ad594x_err_t mfd_ad594x_write_raw16(const struct device *dev, uint16_t val)
{
	const struct mfd_ad594x_config *config = dev->config;

	struct spi_buf tx_buf[] = {
		{
			.buf = &val,
			.len = sizeof(val)
		}
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};

	return spi_write_dt(&config->bus, &tx);
}


ad594x_err_t mfd_ad594x_write_raw(const struct device *dev, uint8_t *val, uint32_t len)
{
	const struct mfd_ad594x_config *config = dev->config;

	struct spi_buf tx_buf[] = {
		{
			.buf = &val,
			.len = len
		}
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};

	return spi_write_dt(&config->bus, &tx);
}

ad594x_err_t mfd_ad594x_transceive_raw(const struct device *dev, uint8_t *tx_ptr, uint8_t *rx_ptr, uint32_t len)
{
	const struct mfd_ad594x_config *config = dev->config;

	struct spi_buf tx_buf[] = {
		{
			.buf = tx_ptr,
			.len = len
		}
	};

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf[] = {
		{
			.buf = rx_ptr,
			.len = len
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 1
	};

	return spi_transceive_dt(&config->bus, &tx, &rx);
}

ad594x_err_t mfd_ad594x_read_reg(const struct device *dev, uint16_t reg, uint32_t *val)
{

	uint8_t msg[] = {SPICMD_SETADDR, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
	int ret;


	ret = mfd_ad594x_write_raw(dev, &msg, 3);
	if (ret < 0) {
		return ret;
	}
	uint8_t write[] = {SPICMD_READREG, 0, 0, 0, 0, 0};
	uint8_t read[6] = {0, 0, 0, 0, 0, 0};
	if((RegAddr>=0x1000)&&(RegAddr<=0x3014)) {
		mfd_ad594x_transceive_raw(dev, &write, &read, 6);
	} else {
		mfd_ad594x_transceive_raw(dev, &write, &read, 4);
	}
	if (ret < 0) {
		return ret;
	}
	*val = sys_be32_to_cpu((uint32_t)read[2] << 24) | ((uint32_t)read[3] << 16) | ((uint32_t)read[4] << 8) | (uint32_t)read[5]);

	return 0;
}

ad594x_err_t mfd_ad594x_read_fifo(const struct device *dev, uint32_t *buffer,  uint32_t readCount)
{
	int ret;
	if (readCount < 3) {
		uint32_t i;
		uint8_t msg[] = {SPICMD_SETADDR, (uint8_t)(REG_AFE_DATAFIFORD >> 8), (uint8_t)(REG_AFE_DATAFIFORD & 0xFF)};
		ret = mfd_ad594x_write_raw(dev, &msg, 3);
		for(i=0;i<uiReadCount;i++)
		{
			uint8_t write[] = {SPICMD_READREG, 0, 0, 0, 0, 0};
			uint8_t read[6] = {0, 0, 0, 0, 0, 0};

			mfd_ad594x_transceive_raw(dev, &write, &read, 6);
			if (ret < 0) {
				return ret;
			}
			buffer[i] = sys_be32_to_cpu((uint32_t)read[2] << 24) | ((uint32_t)read[3] << 16) | ((uint32_t)read[4] << 8) | (uint32_t)read[5]);
		}
	} else {

	}
}

ad594x_err_t mfd_ad594x_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	uint8_t msg[] = {SPICMD_SETADDR, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
	ret = mfd_ad594x_write_raw(dev, &msg, 3);
	if (ret < 0) {
		return ret;
	}

	uint8_t write[] = {SPICMD_WRITEREG, 0, 0, 0, 0};
	if(((RegAddr>=0x1000)&&(RegAddr<=0x3014))) {
		mfd_ad594x_write_raw(dev, &write, 5);
	} else {
		mfd_ad594x_write_raw(dev, &write, 3);
	}

	msg = sys_cpu_to_be32((reg << AD594x_REG_SHIFT_VAL) | (val & write_mask));

	return mfd_ad594x_write_raw32(dev, msg);
}

ad594x_err_t mfd_add592x_software_reset(const struct device *dev)
{
	int ret;
	ret = mfd_ad594x_write_reg(dev, REG_AFECON_SWRSTCON, AD5940_SWRST);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_USEC(200));
	return 0;
}

ad594x_err_t mfd_add592x_hardware_reset(const struct device *dev)
{
	const struct mfd_ad594x_config *config = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	k_sleep(K_USEC(2000));
	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	k_sleep(K_USEC(5000));
	return 0;
}
/**
 * @brief Control most AFE digital and analog block within one register access.
 * @param AfeCtrlSet: A set of blocks that will be controlled select it from @ref AFECTRL_Const Below is two examples to use it.
 *        - AFECTRL_HPREFPWR: Control high power reference(bandgap).
 *        - AFECTRL_WG|AFECTRL_ADCPWR: The OR'ed control set. Control Waveform generator and ADC power.
 * @param State: Enable or disable selected control set signal. Select from @bool
 *        - bFALSE: Disable or power down selected block(s).
 *        - bTRUE:  Enable all selected block(s).
   @return return none.
*/
void  mfd_add592x_afe_ctrl(const struct device *dev, uint32_t AfeCtrlSet, bool State)
{
  /* Check parameters */
  uint32_t tempreg;
  mfd_ad594x_read_reg(dev, REG_AFE_AFECON, &tempreg);
  if (State == bTRUE) {
    /* Clear bits to enable HPREF and ALDOLimit*/
    if (AfeCtrlSet & AFECTRL_HPREFPWR) {
        tempreg &= ~BITM_AFE_AFECON_HPREFDIS;
        AfeCtrlSet &= ~AFECTRL_HPREFPWR;
    }
    if(AfeCtrlSet & AFECTRL_ALDOLIMIT)
    {
      tempreg &= ~BITM_AFE_AFECON_ALDOILIMITEN;
      AfeCtrlSet &= ~AFECTRL_ALDOLIMIT;
    }
    tempreg |= AfeCtrlSet;
  }
  else
  {
    /* Set bits to Disable HPREF and ALDOLimit*/
    if(AfeCtrlSet & AFECTRL_HPREFPWR)
    {
        tempreg |= BITM_AFE_AFECON_HPREFDIS;
        AfeCtrlSet &= ~AFECTRL_HPREFPWR;
    }
    if(AfeCtrlSet & AFECTRL_ALDOLIMIT)
    {
      tempreg |= BITM_AFE_AFECON_ALDOILIMITEN;
      AfeCtrlSet &= ~AFECTRL_ALDOLIMIT;
    }
    tempreg &= ~AfeCtrlSet;
  }
  mfd_ad594x_write_reg(dev, REG_AFE_AFECON, tempreg);
}
/** When LP mode is enabled, some functions are under control of LPMODECON, rather than original registers.  */
/** @warning LPMODE is key protected, this function only takes effect after AD5940_LPModeEnS(bTRUE) */
/**
 * @brief For LP mode, use one register to control most AFE digital and analog block.
 * @details The parameter means the blocks. The selected block will be enabled. All others will be disabled.
 *          The method to enable/disable blocks are defined by register LPMODECON, either by clearing or setting bits.
 * @param EnSet: A set of blocks that will be enabled. Select it from @ref LPMODECTRL_Const. All others not selected in EnSet will be disabled.
 *        - LPMODECTRL_ALDOPWR|LPMODECTRL_HFOSCEN: Turn on ALDO and HFOSC, disable all others.
 *        - LPMODECTRL_ALL: Enable all blocks.
   @return return none.
*/
ad594x_err_t mfd_add592x_lpmode_ctrl(const struct device *dev,uint32_t EnSet)
{
  /* Check parameters */
  uint32_t tempreg;
  uint32_t DisSet;    /* The blocks to be disabled */
  DisSet = LPMODECTRL_ALL & (~EnSet);
  mfd_ad594x_read_reg(dev, REG_AFE_LPMODECON, &tempreg);
  /* Enable selected set */
  {
    /* Clear bits to enable HFOSC, HPREF, ALDO */
    if (EnSet & LPMODECTRL_HFOSCEN) {
        tempreg &= ~BITM_AFE_LPMODECON_HFOSCPD;
        EnSet &= ~LPMODECTRL_HFOSCEN;
    }
    if(EnSet & LPMODECTRL_HPREFPWR)
    {
      tempreg &= ~BITM_AFE_LPMODECON_HPREFDIS;
      EnSet &= ~LPMODECTRL_HPREFPWR;
    }
    if(EnSet & LPMODECTRL_ALDOPWR)
    {
      tempreg &= ~BITM_AFE_LPMODECON_ALDOEN;
      EnSet &= ~LPMODECTRL_ALDOPWR;
    }
    tempreg |= EnSet; /* Set other bits to enable function */
  }
  /* Disable other blocks */
  {
    /* Set bits to disable HFOSC, HPREF, ALDO */
    if (DisSet & LPMODECTRL_HFOSCEN) {
        tempreg |= BITM_AFE_LPMODECON_HFOSCPD;
        DisSet &= ~LPMODECTRL_HFOSCEN;
    }
    if(DisSet & LPMODECTRL_HPREFPWR)
    {
      tempreg |= BITM_AFE_LPMODECON_HPREFDIS;
      DisSet &= ~LPMODECTRL_HPREFPWR;
    }
    if(DisSet & LPMODECTRL_ALDOPWR)
    {
      tempreg |= BITM_AFE_LPMODECON_ALDOEN;
      DisSet &= ~LPMODECTRL_ALDOPWR;
    }
    tempreg &= ~DisSet; /* Clear other bits to disable function */
  }
  mfd_ad594x_write_reg(dev, REG_AFE_LPMODECON, tempreg);

  return AD5940ERR_OK;
}

/**
   @brief Set AFE power mode and system bandwidth include HSDAC, Excitation-buffer, HSTIA and ADC etc.
   @param AfePwr : {AFEPWR_LP, AFEPWR_HP}
          Select parameters from @ref AFEPWR_Const
          - AFEPWR_LP: Set AFE to low power mode
          - AFEPWR_HP: Set AFE to High speed mode to support 200kHz.
   @param AfeBw : {AFEBW_AUTOSET, AFEBW_50KHZ, AFEBW_100KHZ, AFEBW_250KHZ}
          - AFEBW_AUTOSET: Set the bandwidth automatically based on WGFCW frequency word.
          - AFEBW_50KHZ: Set system bandwidth to 50kHz.
          - AFEBW_100KHZ: Set system bandwidth to 100kHz.
          - AFEBW_250KHZ: Set system bandwidth to 250kHz.
   @return return none.
*/
void mfd_add592x_afe_pwr_bw(const struct device *dev, uint32_t AfePwr, uint32_t AfeBw)
{
  //check parameters
  uint32_t tempreg;
  tempreg = AfePwr;
  tempreg |= AfeBw << BITP_AFE_PMBW_SYSBW;
  mfd_ad594x_write_reg(dev, REG_AFE_PMBW, tempreg);
}

/**
   @brief Configure reference buffer include 1.8V/1.1V high/low power buffers.
   @param pBufCfg :Pointer to buffer configure structure;
   @return return none.
*/
void mfd_add592x_ref_cfg(const struct device *dev, afe_ref_cfg_t *pBufCfg)
{
  uint32_t tempreg;

  /* HP Reference(bandgap) */
  mfd_ad594x_read_reg(dev, REG_AFE_AFECON, &tempreg);
  tempreg &= ~BITM_AFE_AFECON_HPREFDIS;
  if(pBufCfg->HpBandgapEn == bFALSE)
    tempreg |= BITM_AFE_AFECON_HPREFDIS;
  mfd_ad594x_write_reg(dev, REG_AFE_AFECON, tempreg);
  /* Reference buffer configure */
  mfd_ad594x_read_reg(dev, REG_AFE_BUFSENCON, &tempreg);
  if(pBufCfg->Hp1V8BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8HPADCEN;
  if(pBufCfg->Hp1V1BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P1HPADCEN;
  if(pBufCfg->Lp1V8BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8LPADCEN;
  if(pBufCfg->Lp1V1BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P1LPADCEN;
  if(pBufCfg->Hp1V8ThemBuff == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8THERMSTEN;
  if(pBufCfg->Hp1V8Ilimit == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8HPADCILIMITEN;
  if(pBufCfg->Disc1V8Cap == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8HPADCCHGDIS;
  if(pBufCfg->Disc1V1Cap == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P1LPADCCHGDIS;
  mfd_ad594x_write_reg(dev,REG_AFE_BUFSENCON, tempreg);

  /* LPREFBUFCON */
  tempreg = 0;
  if(pBufCfg->LpRefBufEn == bFALSE)
    tempreg |= BITM_AFE_LPREFBUFCON_LPBUF2P5DIS;
  if(pBufCfg->LpBandgapEn == bFALSE)
    tempreg |= BITM_AFE_LPREFBUFCON_LPREFDIS;
  if(pBufCfg->LpRefBoostEn == bTRUE)
    tempreg |= BITM_AFE_LPREFBUFCON_BOOSTCURRENT;
  mfd_ad594x_write_reg(dev,REG_AFE_LPREFBUFCON, tempreg);
}
/**
 * @} End of AFE_Control_Functions
 * @} End of AFE_Control
 * */



/**
 * @defgroup Low_Power_Loop
 * @brief The low power loop.
 * @{
 *    @defgroup Low_Power_Loop_Functions
 *    @{
*/

/**
   @brief Configure low power loop include LPDAC LPAmp(PA and TIA)
   @param pLpLoopCfg: Pointer to configure structure;
   @return return none.
*/
void mfd_ad594x_lp_loop_cfg(const struct device *dev, lp_loop_cfg_t *pLpLoopCfg)
{
  mfd_ad594x_lp_dac_cfg(&pLpLoopCfg->LpDacCfg);
  mfd_ad594x_lp_amp_cfg(&pLpLoopCfg->LpAmpCfg);
}

/**
   @brief Initialize LPDAC
   @param pLpDacCfg: Pointer to configuration structure
   @return return none.
*/
void mfd_ad594x_lp_dac_cfg(const struct device *dev, lp_dac_cfg_t *pLpDacCfg)
{
  uint32_t tempreg;
  tempreg = 0;
  tempreg = (pLpDacCfg->LpDacSrc)<<BITP_AFE_LPDACCON0_WAVETYPE;
  tempreg |= (pLpDacCfg->LpDacVzeroMux)<<BITP_AFE_LPDACCON0_VZEROMUX;
  tempreg |= (pLpDacCfg->LpDacVbiasMux)<<BITP_AFE_LPDACCON0_VBIASMUX;
  tempreg |= (pLpDacCfg->LpDacRef)<<BITP_AFE_LPDACCON0_REFSEL;
  if(pLpDacCfg->DataRst == bFALSE)
    tempreg |= BITM_AFE_LPDACCON0_RSTEN;
  if(pLpDacCfg->PowerEn == bFALSE)
    tempreg |= BITM_AFE_LPDACCON0_PWDEN;
  if(pLpDacCfg->LpdacSel == LPDAC0)
  {
    mfd_ad594x_write_reg(dev, REG_AFE_LPDACCON0, tempreg);
    mfd_ad594x_lp_dac0_write(dev, pLpDacCfg->DacData12Bit, pLpDacCfg->DacData6Bit);
    mfd_ad594x_write_reg(dev, REG_AFE_LPDACSW0, pLpDacCfg->LpDacSW|BITM_AFE_LPDACSW0_LPMODEDIS);  /* Overwrite LPDACSW settings. On Si1, this register is not accessible. */
  }
  else
  {
    mfd_ad594x_write_reg(dev, REG_AFE_LPDACCON1, tempreg);
    mfd_ad594x_lp_dac1_write(dev, pLpDacCfg->DacData12Bit, pLpDacCfg->DacData6Bit);
    mfd_ad594x_write_reg(dev, REG_AFE_LPDACSW1, pLpDacCfg->LpDacSW|BITM_AFE_LPDACSW0_LPMODEDIS);  /* Overwrite LPDACSW settings. On Si1, this register is not accessible. */
  }
}

/**
   @brief Write LPDAC data
   @param Data12Bit: 12Bit DAC data
   @param Data6Bit: 6Bit DAC data
   @return return none.
*/
void mfd_ad594x_lp_dac_dat_write(const struct device *dev, uint16_t Data12Bit, uint8_t Data6Bit)
{
  /* Check parameter */
  Data6Bit &= 0x3f;
  Data12Bit &= 0xfff;
  mfd_ad594x_write_reg(dev, REG_AFE_LPDACDAT0, ((uint32_t)Data6Bit<<12)|Data12Bit);
}

/**
   @brief Write LPDAC0 data
   @param Data12Bit: 12Bit DAC data
   @param Data6Bit: 6Bit DAC data
   @return return none.
*/
void mfd_ad594x_lp_dac0_write(const struct device *dev, uint16_t Data12Bit, uint8_t Data6Bit)
{
  /* Check parameter */
  Data6Bit &= 0x3f;
  Data12Bit &= 0xfff;
  mfd_ad594x_write_reg(dev, REG_AFE_LPDACDAT0, ((uint32_t)Data6Bit<<12)|Data12Bit);
}

/**
   @brief Write LPDAC1 data
   @param Data12Bit: 12Bit DAC data
   @param Data6Bit: 6Bit DAC data
   @return return none.
*/
void mfd_ad594x_lp_dac1_write(const struct device *dev, uint16_t Data12Bit, uint8_t Data6Bit)
{
  /* Check parameter */
  Data6Bit &= 0x3f;
  Data12Bit &= 0xfff;
  mfd_ad594x_write_reg(dev, REG_AFE_LPDACDAT1, ((uint32_t)Data6Bit<<12)|Data12Bit);
}

/**
   @brief Initialize LP TIA and PA
   @param pLpAmpCfg: Pointer to configuration structure
   @return return none.
*/
void mfd_ad594x_lp_amp_cfg(const struct device *dev, lp_amp_cfg_t *pLpAmpCfg)
{
  //check parameters
  uint32_t tempreg;

  tempreg = 0;
  if(pLpAmpCfg->LpPaPwrEn == bFALSE)
    tempreg |= BITM_AFE_LPTIACON0_PAPDEN;
  if(pLpAmpCfg->LpTiaPwrEn == bFALSE)
    tempreg |= BITM_AFE_LPTIACON0_TIAPDEN;
  if(pLpAmpCfg->LpAmpPwrMod == LPAMPPWR_HALF)
    tempreg |= BITM_AFE_LPTIACON0_HALFPWR;
  else
  {
    tempreg |= pLpAmpCfg->LpAmpPwrMod<<BITP_AFE_LPTIACON0_IBOOST;
  }
  tempreg |= pLpAmpCfg->LpTiaRtia<<BITP_AFE_LPTIACON0_TIAGAIN;
  tempreg |= pLpAmpCfg->LpTiaRload<<BITP_AFE_LPTIACON0_TIARL;
  tempreg |= pLpAmpCfg->LpTiaRf<<BITP_AFE_LPTIACON0_TIARF;
  if(pLpAmpCfg->LpAmpSel == LPAMP0)
  {
    mfd_ad594x_write_reg(dev, REG_AFE_LPTIACON0, tempreg);
    mfd_ad594x_write_reg(dev, REG_AFE_LPTIASW0, pLpAmpCfg->LpTiaSW);
  }
  else
  {
    mfd_ad594x_write_reg(dev, REG_AFE_LPTIACON1, tempreg);
    mfd_ad594x_write_reg(dev, REG_AFE_LPTIASW1, pLpAmpCfg->LpTiaSW);
  }
}
/**
 * @} Low_Power_Loop_Functions
 * @} Low_Power_Loop
*/

/**
 * @defgroup LPMode_Block_Functions
 * @{
*/
/**
 * @brief Enter or leave LPMODE.
 * @details Once enter this mode, some registers are collected together to a new register so we can
 *          Control most blocks with in one register. The so called LPMODE has nothing to do with AD5940 power.
 * @return return AD5940ERR_OK
**/
int mfd_ad594x_lpmode_toggle(const struct device *dev, bool LPModeEn)
{
  if(LPModeEn == true)
     return mfd_ad594x_write_reg(dev, REG_AFE_LPMODEKEY, KEY_LPMODEKEY);  /* Enter LP mode by right key. */
  else
     return mfd_ad594x_write_reg(dev, REG_AFE_LPMODEKEY, 0); /* Write wrong key to exit LP mode */
}

/**
 * @brief Select system clock source for LPMODE.
 * @note Only in LP Mode, this operation takes effect. Enter LPMODE by function @ref AD5940_LPModeEnS.
 * @param LPModeClk: Select from @ref LPMODECLK_Const
 *       - LPMODECLK_LFOSC: Select LFOSC 32kHz for system clock
 *       - LPMODECLK_HFOSC: Select HFOSC 16MHz/32MHz for system clock
 * @return none.
*/
void mfd_ad594x_lpmode_clksel(const struct device *dev, uint32_t lp_mode_clk)
{
  mfd_ad594x_write_reg(dev, REG_AFE_LPMODECLKSEL, lp_mode_clk);
}

/**
 * @brief Set ADC Repeat convert function number. Turn off ADC automatically after Number samples of ADC raw data are ready
 * @param Number: Specify after how much ADC raw data need to sample before shutdown ADC
 * @return return none.
*/
void mfd_ad594x_adc_repeat_cfg(const struct device *dev, uint32_t Number)
{
  //check parameter if(number<255)
  mfd_ad594x_write_reg(dev, REG_AFE_REPEATADCCNV, Number<<BITP_AFE_REPEATADCCNV_NUM);
}

/**
 * @} LPMode_Block_Functions
*/

/**
 * @brief Enter sleep mode key to unlock it or enter incorrect key to lock it. \
 *        Once key is unlocked, it will always be effect until manually lock it
 * @param SlpKey : {SLPKEY_UNLOCK, SLPKEY_LOCK}
          - SLPKEY_UNLOCK Unlock Key so we can enter sleep(or called hibernate) mode.
          - SLPKEY_LOCK Lock key so AD5940 is prohibited to enter sleep mode.
   @return return none.
*/
void mfd_ad594x_sleepkey_ctrl(const struct device *dev, uint32_t slp_key)
{
  mfd_ad594x_write_reg(dev, REG_AFE_SEQSLPLOCK, slp_key);
}

/**
 * @brief Put AFE to hibernate.
 * @details This will only take effect when SLP_KEY has been unlocked. Use function @ref AD5940_SleepKeyCtrlS to enter correct key.
 * @return return none.
*/
void mfd_ad594x_enter_sleep(const struct device *dev)
{
   mfd_ad594x_write_reg(dev, REG_AFE_SEQTRGSLP, 0);
   mfd_ad594x_write_reg(dev, REG_AFE_SEQTRGSLP, 1);
}

/**
 * @brief Turn off LP-Loop and put AFE to hibernate mode;
 * @details By function @ref AD5940_EnterSleepS, we can put most blocks to hibernate mode except LP block.
 *          This function will shut down LP block and then enter sleep mode.
 * @return return none.
*/
void mfd_ad594x_hibernate(const struct device *dev)
{
  /* Turn off LPloop related blocks which are not controlled automatically by hibernate operation */
  afe_ref_cfg_t aferef_cfg;
  lp_loop_cfg_t lp_loop;
  /* Turn off LP-loop manually because it's not affected by sleep/hibernate mode */
  mfd_ad594x_struct_init(&aferef_cfg, sizeof(aferef_cfg));
  mfd_ad594x_struct_init(&lp_loop, sizeof(lp_loop));
  mfd_add592x_ref_cfg(dev, &aferef_cfg);
  mfd_ad594x_lp_loop_cfg(dev, &lp_loop);
  mfd_ad594x_sleepkey_ctrl(dev, SLPKEY_UNLOCK);  /* Unlock the key */
  mfd_ad594x_enter_sleep(dev);  /* Enter Hibernate */
}

/**
 * @brief Try to wakeup AD5940 by read register.
 * @details Any SPI operation can wakeup AD5940. AD5940_Initialize must be called to enable this function.
 * @param TryCount Specify how many times we will read register. Zero or negative number means always waiting here.
 * @return How many times register is read. If returned value is bigger than TryCount, it means wakeup failed.
*/
uint32_t mfd_ad594x_wake_up(const struct device *dev, int32_t TryCount)
{
  uint32_t count = 0;
  while(1)
  {
    count++;
    uint32_t tempreg;
    mfd_ad594x_read_reg(dev, REG_AFE_AFECON, &tempreg);
    if(tempreg == AD5940_ADIID)
      break;    /* Succeed */
    if(TryCount<=0)
      continue; /* Always try to wakeup AFE */

    if(count > TryCount)
      break;    /* Failed */
  }
  return count;
}

/**
 * @brief Read ADIID register, the value for current version is @ref AD5940_ADIID
 * @return return none.
*/
uint32_t mfd_ad594x_get_adiid(const struct device *dev)
{
	uint32_t tempreg;
  	mfd_ad594x_read_reg(dev, REG_AFE_AFECON, &tempreg);
	return tempreg;
}

/**
 * @brief Read CHIPID register, the value for current version is 0x5501.
 * @return return none.
*/
uint32_t mfd_ad594x_get_chip_id(const struct device *dev)
{
	uint32_t tempreg;
  	mfd_ad594x_read_reg(dev, REG_AFECON_CHIPID, &tempreg);
	return tempreg;
}

/**
  @brief Initialize Structure members to zero
  @param struct: Pointer to the structure.
  @param size: The structure size in Byte.
  @return Return None.
**/
void mfd_add592x_struct_init(void *struct, uint32_t size)
{
  memset(struct, 0, size);
}

static int mfd_ad594x_init(const struct device *dev)
{
	const struct mfd_ad594x_config *config = dev->config;
	int ret;

	if (!spi_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
	if (ret < 0) {
		return ret;
	}

	ret = mfd_add592x_software_reset(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#define MFD_AD594x_DEFINE(inst)							\
	static const struct mfd_ad594x_config mfd_ad594x_config_##inst = {	\
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),		\
		.bus = SPI_DT_SPEC_INST_GET(inst, AD594x_SPI_SPEC_CONF, 0),	\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, mfd_ad594x_init, NULL,			\
			      NULL,						\
			      &mfd_ad594x_config_##inst,			\
			      POST_KERNEL,					\
			      CONFIG_MFD_INIT_PRIORITY,				\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_AD594x_DEFINE);
