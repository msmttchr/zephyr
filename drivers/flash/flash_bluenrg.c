/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2017 BayLibre, SAS.
 * Copyright (c) 2019 Centaur Analytics, Inc
 * Copyright (c) 2023 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define DT_DRV_COMPAT st_stm32_flash_controller

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/stm32_flash_api_extensions.h>
#include <zephyr/init.h>
#include <soc.h>
#include <rf_driver_ll_bus.h>
#include <rf_driver_ll_rcc.h>
#include <zephyr/logging/log.h>

#include "flash_bluenrg.h"
#include "bluenrg_hsem.h"

LOG_MODULE_REGISTER(flash_stm32, CONFIG_FLASH_LOG_LEVEL);

/* Let's wait for double the max erase time to be sure that the operation is
 * completed.
 */
#define STM32_FLASH_TIMEOUT	\
	(2 * DT_PROP(DT_INST(0, st_stm32_nv_flash), max_erase_time))


#define FLASH_PAGE_SIZE DT_PROP(DT_INST(0, st_stm32_nv_flash), erase_block_size)

static int do_flash_command (FLASH_TypeDef *regs, uint32_t code)
{
	uint32_t irqstat;

	regs->IRQRAW = 0x1F;

	regs->COMMAND = code;
	/* Wait for command start */
	while (1) {
		irqstat = regs->IRQRAW;
		if (irqstat & FLASH_IRQRAW_CMDSTART_RIS) {
         	        regs->IRQRAW = FLASH_IRQRAW_CMDSTART_RIS;
			break;
		} else if (irqstat & FLASH_IRQRAW_ILLCMD_RIS) {
		        return -EINVAL;
		} else if (irqstat & FLASH_IRQRAW_CMDERR_RIS) {
		        return -EBUSY;
		}
	}
	/* Wait for command done */
	while (1) {
		irqstat = regs->IRQRAW;
		if (irqstat & FLASH_IRQRAW_CMDDONE_RIS) {
         	        regs->IRQRAW = FLASH_IRQRAW_CMDDONE_RIS;
			break;
		}
	}
	return 0;
}

static const struct flash_parameters flash_stm32_parameters = {
	.write_block_size = FLASH_STM32_WRITE_BLOCK_SIZE,
	/* Some SoCs (L0/L1) use an EEPROM under the hood. Distinguish
	 * between them based on the presence of the PECR register. */
#if defined(FLASH_PECR_ERASE)
	.erase_value = 0,
#else
	.erase_value = 0xff,
#endif
};

static int flash_stm32_write_protection(const struct device *dev, bool enable);

bool __weak flash_stm32_valid_range(const struct device *dev, off_t offset,
				    uint32_t len, bool write)
{
	if (write && !flash_stm32_valid_write(offset, len)) {
		return false;
	}
	return flash_stm32_range_exists(dev, offset, len);
}

int __weak flash_stm32_check_configuration(void)
{
	return 0;
}

static void flash_stm32_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	static struct flash_pages_layout bluenrg_flash_layout = {
		.pages_count = 0,
		.pages_size = 0,
	};

	ARG_UNUSED(dev);

	if (bluenrg_flash_layout.pages_count == 0) {
	        uint32_t flash_size  = ((FLASH_STM32_REGS(dev)->SIZE & 0xFFFF) + 1) << 2;
		bluenrg_flash_layout.pages_count = flash_size / FLASH_PAGE_SIZE;
		bluenrg_flash_layout.pages_size = FLASH_PAGE_SIZE;
	}

	*layout = &bluenrg_flash_layout;
	*layout_size = 1;
}

static int erase_page(const struct device *dev, unsigned int offset)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);
	int rc;

	LOG_DBG("Erase page %d\n", offset >> 11);
	regs->ADDRESS = (offset  >> 2) & 0xFE00;
	rc = do_flash_command(regs, 0x11);
	
	return rc;
}

static int flash_stm32_block_erase_loop(const struct device *dev,
				 unsigned int offset,
				 unsigned int len)
{
	unsigned int address = offset;
	int rc = 0;

	for (; address <= offset + len - 1 ; address += LL_FLASH_PAGE_SIZE) {
		rc = erase_page(dev, address);
		if (rc < 0) {
			break;
		}
	}

	return rc;
}

static int write_qword(const struct device *dev, off_t offset, const uint32_t *buff)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);
	volatile uint32_t *flash = (uint32_t *)(offset
						+ CONFIG_FLASH_BASE_ADDRESS);

	/* Check if this double word is erased */
	if (((flash[0] != 0xFFFFFFFFUL) && (flash[0] != buff[0])) ||
	    ((flash[1] != 0xFFFFFFFFUL) && (flash[1] != buff[1])) ||
	    ((flash[2] != 0xFFFFFFFFUL) && (flash[2] != buff[2])) ||
	    ((flash[3] != 0xFFFFFFFFUL) && (flash[3] != buff[3]))) {
		LOG_ERR("Word at offs %ld not erased", (long)offset);
		return -EIO;
	}

	regs->ADDRESS = (offset >> 2) & 0xffff;
	regs->DATA0 = buff[0];
	regs->DATA1 = buff[1];
	regs->DATA2 = buff[2];
	regs->DATA3 = buff[3];
	return do_flash_command(regs, 0xCC);
}

static int flash_stm32_write_range(const struct device *dev, unsigned int offset,
			    const void *data, unsigned int len)
{
	int i, rc = 0;
	for (i = 0; i < len; i += 16) {
		rc = write_qword(dev, offset + i, ((const uint32_t *) data + (i>>2)));
		if (rc < 0) {
			break;
		}
	}

	return rc;
}


#if defined(CONFIG_MULTITHREADING)
/*
 * This is named flash_stm32_sem_take instead of flash_stm32_lock (and
 * similarly for flash_stm32_sem_give) to avoid confusion with locking
 * actual flash pages.
 */
static inline void _flash_stm32_sem_take(const struct device *dev)
{
	k_sem_take(&FLASH_STM32_PRIV(dev)->sem, K_FOREVER);
	z_stm32_hsem_lock(CFG_HW_FLASH_SEMID, HSEM_LOCK_WAIT_FOREVER);
}

static inline void _flash_stm32_sem_give(const struct device *dev)
{
	z_stm32_hsem_unlock(CFG_HW_FLASH_SEMID);
	k_sem_give(&FLASH_STM32_PRIV(dev)->sem);
}

#define flash_stm32_sem_init(dev) k_sem_init(&FLASH_STM32_PRIV(dev)->sem, 1, 1)
#define flash_stm32_sem_take(dev) _flash_stm32_sem_take(dev)
#define flash_stm32_sem_give(dev) _flash_stm32_sem_give(dev)
#else
#define flash_stm32_sem_init(dev)
#define flash_stm32_sem_take(dev)
#define flash_stm32_sem_give(dev)
#endif

int flash_stm32_wait_flash_idle(const struct device *dev)
{
	return 0;
}

static void flash_stm32_flush_caches(const struct device *dev,
				     off_t offset, size_t len)
{
#if defined(CONFIG_SOC_SERIES_STM32F0X) || defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || defined(CONFIG_SOC_SERIES_STM32L5X) || \
	defined(CONFIG_SOC_SERIES_STM32U5X) || defined(CONFIG_SOC_SERIES_STM32H5X)
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(len);
#elif defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X)
	ARG_UNUSED(offset);
	ARG_UNUSED(len);

	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

	if (regs->ACR & FLASH_ACR_DCEN) {
		regs->ACR &= ~FLASH_ACR_DCEN;
		regs->ACR |= FLASH_ACR_DCRST;
		regs->ACR &= ~FLASH_ACR_DCRST;
		regs->ACR |= FLASH_ACR_DCEN;
	}
#elif defined(CONFIG_SOC_SERIES_STM32F7X)
	SCB_InvalidateDCache_by_Addr((uint32_t *)(CONFIG_FLASH_BASE_ADDRESS
						  + offset), len);
#endif
}

static int flash_stm32_read(const struct device *dev, off_t offset,
			    void *data,
			    size_t len)
{
	if (!flash_stm32_valid_range(dev, offset, len, false)) {
		LOG_ERR("Read range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("Read offset: %ld, len: %zu", (long int) offset, len);

	memcpy(data, (uint8_t *) CONFIG_FLASH_BASE_ADDRESS + offset, len);

	return 0;
}

static int flash_stm32_erase(const struct device *dev, off_t offset,
			     size_t len)
{
	int rc;

	if (!flash_stm32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Erase range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_stm32_sem_take(dev);

	LOG_DBG("Erase offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_stm32_write_protection(dev, false);
	if (rc == 0) {
		rc = flash_stm32_block_erase_loop(dev, offset, len);
	}

	flash_stm32_flush_caches(dev, offset, len);

	int rc2 = flash_stm32_write_protection(dev, true);

	if (!rc) {
		rc = rc2;
	}

	flash_stm32_sem_give(dev);

	return rc;
}

static int flash_stm32_write(const struct device *dev, off_t offset,
			     const void *data, size_t len)
{
	int rc;

	if (!flash_stm32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Write range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_stm32_sem_take(dev);

	LOG_DBG("Write offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_stm32_write_protection(dev, false);
	if (rc == 0) {
		rc = flash_stm32_write_range(dev, offset, data, len);
	}

	int rc2 = flash_stm32_write_protection(dev, true);

	if (!rc) {
		rc = rc2;
	}

	flash_stm32_sem_give(dev);

	return rc;
}

static int flash_stm32_write_protection(const struct device *dev, bool enable)
{

	int rc = 0;

	return rc;
}

int flash_stm32_option_bytes_lock(const struct device *dev, bool enable)
{

#if defined(FLASH_OPTCR_OPTLOCK) /* F2, F4, F7 and H7 */
	if (enable) {
		regs->OPTCR |= FLASH_OPTCR_OPTLOCK;
	} else if (regs->OPTCR & FLASH_OPTCR_OPTLOCK) {
		regs->OPTKEYR = FLASH_OPT_KEY1;
		regs->OPTKEYR = FLASH_OPT_KEY2;
	}
#else
	int rc;

	/* Unlock CR/PECR/NSCR register if needed. */
	if (!enable) {
		rc = flash_stm32_write_protection(dev, false);
		if (rc) {
			return rc;
		}
	}
#if defined(FLASH_CR_OPTWRE)	  /* F0, F1 and F3 */
	if (enable) {
		regs->CR &= ~FLASH_CR_OPTWRE;
	} else if (!(regs->CR & FLASH_CR_OPTWRE)) {
		regs->OPTKEYR = FLASH_OPTKEY1;
		regs->OPTKEYR = FLASH_OPTKEY2;
	}
#elif defined(FLASH_CR_OPTLOCK)	  /* G0, G4, L4, WB and WL */
	if (enable) {
		regs->CR |= FLASH_CR_OPTLOCK;
	} else if (regs->CR & FLASH_CR_OPTLOCK) {
		regs->OPTKEYR = FLASH_OPTKEY1;
		regs->OPTKEYR = FLASH_OPTKEY2;
	}
#elif defined(FLASH_PECR_OPTLOCK) /* L0 and L1 */
	if (enable) {
		regs->PECR |= FLASH_PECR_OPTLOCK;
	} else if (regs->PECR & FLASH_PECR_OPTLOCK) {
		regs->OPTKEYR = FLASH_OPTKEY1;
		regs->OPTKEYR = FLASH_OPTKEY2;
	}
#elif defined(FLASH_NSCR_OPTLOCK) /* L5 and U5 */
	if (enable) {
		regs->NSCR |= FLASH_NSCR_OPTLOCK;
	} else if (regs->NSCR & FLASH_NSCR_OPTLOCK) {
		regs->OPTKEYR = FLASH_OPTKEY1;
		regs->OPTKEYR = FLASH_OPTKEY2;
	}
#elif defined(FLASH_NSCR1_OPTLOCK) /* WBA */
	if (enable) {
		regs->NSCR1 |= FLASH_NSCR1_OPTLOCK;
	} else if (regs->NSCR1 & FLASH_NSCR1_OPTLOCK) {
		regs->OPTKEYR = FLASH_OPTKEY1;
		regs->OPTKEYR = FLASH_OPTKEY2;
	}
#endif
	/* Lock CR/PECR/NSCR register if needed. */
	if (enable) {
		rc = flash_stm32_write_protection(dev, true);
		if (rc) {
			return rc;
		}
	}
#endif

	if (enable) {
		LOG_DBG("Option bytes locked");
	} else {
		LOG_DBG("Option bytes unlocked");
	}

	return 0;
}

#if defined(CONFIG_FLASH_STM32_BLOCK_REGISTERS)
static int flash_stm32_control_register_disable(const struct device *dev)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

#if defined(FLASH_CR_LOCK) /* F0, F1, F2, F3, F4, F7, L4, G0, G4, H7, WB, WL   \
			    */
	/*
	 * Access to control register can be disabled by writing wrong key to
	 * the key register. Option register will remain disabled until reset.
	 * Writing wrong key causes a bus fault, so we need to set FAULTMASK to
	 * disable faults, and clear bus fault pending bit before enabling them
	 * again.
	 */
	regs->CR |= FLASH_CR_LOCK;

	__set_FAULTMASK(1);
	regs->KEYR = 0xffffffff;

	/* Clear Bus Fault pending bit */
	SCB->SHCSR &= ~SCB_SHCSR_BUSFAULTPENDED_Msk;
	__set_FAULTMASK(0);

	return 0;
#else
	ARG_UNUSED(regs);

	return -ENOTSUP;
#endif
}

static int flash_stm32_option_bytes_disable(const struct device *dev)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);

#if defined(FLASH_OPTCR_OPTLOCK) /* F2, F4, F7 and H7 */
	/*
	 * Access to option register can be disabled by writing wrong key to
	 * the key register. Option register will remain disabled until reset.
	 * Writing wrong key causes a bus fault, so we need to set FAULTMASK to
	 * disable faults, and clear bus fault pending bit before enabling them
	 * again.
	 */
	regs->OPTCR |= FLASH_OPTCR_OPTLOCK;

	__set_FAULTMASK(1);
	regs->OPTKEYR = 0xffffffff;

	/* Clear Bus Fault pending bit */
	SCB->SHCSR &= ~SCB_SHCSR_BUSFAULTPENDED_Msk;
	__set_FAULTMASK(0);

	return 0;
#else
	ARG_UNUSED(regs);

	return -ENOTSUP;
#endif
}
#endif /* CONFIG_FLASH_STM32_BLOCK_REGISTERS */

static const struct flash_parameters *
flash_stm32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_stm32_parameters;
}

#ifdef CONFIG_FLASH_EX_OP_ENABLED
static int flash_stm32_ex_op(const struct device *dev, uint16_t code,
			     const uintptr_t in, void *out)
{
	int rv = -ENOTSUP;

	flash_stm32_sem_take(dev);

	switch (code) {
#if defined(CONFIG_FLASH_STM32_WRITE_PROTECT)
	case FLASH_STM32_EX_OP_SECTOR_WP:
		rv = flash_stm32_ex_op_sector_wp(dev, in, out);
		break;
#endif /* CONFIG_FLASH_STM32_WRITE_PROTECT */
#if defined(CONFIG_FLASH_STM32_READOUT_PROTECTION)
	case FLASH_STM32_EX_OP_RDP:
		rv = flash_stm32_ex_op_rdp(dev, in, out);
		break;
#endif /* CONFIG_FLASH_STM32_READOUT_PROTECTION */
#if defined(CONFIG_FLASH_STM32_BLOCK_REGISTERS)
	case FLASH_STM32_EX_OP_BLOCK_OPTION_REG:
		rv = flash_stm32_option_bytes_disable(dev);
		break;
	case FLASH_STM32_EX_OP_BLOCK_CONTROL_REG:
		rv = flash_stm32_control_register_disable(dev);
		break;
#endif /* CONFIG_FLASH_STM32_BLOCK_REGISTERS */
	}

	flash_stm32_sem_give(dev);

	return rv;
}
#endif

static struct flash_stm32_priv flash_data = {
	.regs = (FLASH_TypeDef *) DT_INST_REG_ADDR(0),
	/* Getting clocks information from device tree description depending
	 * on the presence of 'clocks' property.
	 */
#if DT_INST_NODE_HAS_PROP(0, clocks)
	.pclken = {
		.enr = DT_INST_CLOCKS_CELL(0, bits),
		.bus = DT_INST_CLOCKS_CELL(0, bus),
	}
#endif
};
#if defined(CONFIG_FLASH_STM32_WRITE_PROTECT)
static int flash_stm32_get_wp_sectors(const struct device *dev,
				      uint32_t *protected_sectors);

typedef struct segment_protection_s {
  uint8_t offset;
  uint8_t size;
} __packed segment_protection_t;

static int mask_to_segments(uint32_t mask, segment_protection_t *segments, uint32_t num_segments)
{
  int i = 0;
  int n_segs = 0;

  while (1) {
    /* Find new segment */
    while (!(mask & (1 << i)) && (i < 32)) i++;
    if (i < 32) {
      int start = i;
      /* New segment size */
      while ((mask & (1 << i)) && (i < 32)) i++;
      if (n_segs >= num_segments) {
	/* Too many segments */
	return -EINVAL;
      }
      segments[n_segs].offset = start;
      segments[n_segs].size = i - start;
    } else {
      break;
    }
  }
  return 0;
}

static int flash_stm32_update_wp_sectors(const struct device *dev,
					 uint32_t unprotected_sectors,
					 uint32_t protected_sectors)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);
	segment_protection_t segment_protection[4] = {0,};
	uint32_t current_mask, new_mask;
	int rc;

	rc = flash_stm32_get_wp_sectors(dev, &current_mask);
	if (!rc) {
	  return rc;
	}
	new_mask = (current_mask | protected_sectors) & ~unprotected_sectors;

	if (new_mask == current_mask)
	  return 0;

	rc = mask_to_segments(new_mask, segment_protection, 4);

	if (rc) {
		return -EINVAL;
	}

	regs->PAGEPROT0 = *(unsigned int *) &segment_protection[0];
	regs->PAGEPROT1 = *(unsigned int *) &segment_protection[2];

	return 0;
}

static int flash_stm32_get_wp_sectors(const struct device *dev,
			       uint32_t *protected_sectors)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);
	segment_protection_t segment_protection[4];
	int i;

	/* Read registers */
	*(unsigned int *) &segment_protection[0] = regs->PAGEPROT0;
	*(unsigned int *) &segment_protection[2] = regs->PAGEPROT1;

	for (i = 0; i < (sizeof(segment_protection)/sizeof(segment_protection_t)); i++) {
	  uint32_t bits;
	  /* Clear reserved bits */
	  segment_protection[i].offset &= ~0x80;
	  segment_protection[i].size &= ~0x80;
	  /* Get bitmask */
	  bits = (1 << segment_protection[i].size) - 1;
	  *protected_sectors = (bits << segment_protection[i].offset);
	}

	return 0;
}
#endif /* CONFIG_FLASH_STM32_WRITE_PROTECT */

#if defined(CONFIG_FLASH_STM32_READOUT_PROTECTION)
static int flash_stm32_get_rdp(const struct device *dev, bool *enabled)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);
	uint32_t current_level;

	current_level = regs->SIZE;

	*enabled = (current_level & FLASH_FLASH_SIZE_FLASH_SECURE);

	return 0;
}

static int flash_stm32_update_rdp(const struct device *dev, bool enable)
{
	FLASH_TypeDef *regs = FLASH_STM32_REGS(dev);
	bool enabled;
	int rc;

	rc = flash_stm32_get_rdp(dev, &enabled);

	if (!rc) {
	  if (enabled && !enable) {
		do_flash_command(regs, 0x22);
	    /* Flash content is lost */
	  } else if (!enabled && enable) {
	    regs->DATA0 = 0xAAAAAAAA;
	    regs->DATA1 = 0xAAAAAAAA;
	    regs->DATA2 = 0xC7EF584D;
	    regs->DATA3 = 0xB3A21096;
		do_flash_command(regs, 0xFF);
	    /* This is effective at the next reset */
	  }
	}

	return rc;
}

int flash_stm32_ex_op_sector_wp(const struct device *dev, const uintptr_t in,
				void *out)
{
	const struct flash_stm32_ex_op_sector_wp_in *request =
		(const struct flash_stm32_ex_op_sector_wp_in *)in;
	struct flash_stm32_ex_op_sector_wp_out *result =
		(struct flash_stm32_ex_op_sector_wp_out *)out;
	uint32_t disable_mask;
	int rc = 0, rc2 = 0;
#ifdef CONFIG_USERSPACE
	bool syscall_trap = z_syscall_trap();
#endif

	if (request != NULL) {
#ifdef CONFIG_USERSPACE
		struct flash_stm32_ex_op_sector_wp_in in_copy;

		if (syscall_trap) {
			Z_OOPS(z_user_from_copy(&in_copy, request,
						sizeof(in_copy)));
			request = &in_copy;
		}
#endif
		disable_mask = request->disable_mask;

		if (!IS_ENABLED(
			    CONFIG_FLASH_STM32_WRITE_PROTECT_DISABLE_PREVENTION)) {
			disable_mask = 0;
		}

		if (rc == 0) {
			rc = flash_stm32_update_wp_sectors(
				dev, request->disable_mask, request->enable_mask);
		}
	}

	if (result != NULL) {
#ifdef CONFIG_USERSPACE
		struct flash_stm32_ex_op_sector_wp_out out_copy;

		if (syscall_trap) {
			result = &out_copy;
		}
#endif
		rc2 = flash_stm32_get_wp_sectors(dev, &result->protected_mask);
		if (!rc) {
			rc = rc2;
		}

#ifdef CONFIG_USERSPACE
		if (syscall_trap) {
			Z_OOPS(z_user_to_copy(out, result, sizeof(out_copy)));
		}
#endif
	}

	return rc;
}
#endif /* CONFIG_FLASH_STM32_WRITE_PROTECT */

#if defined(CONFIG_FLASH_STM32_READOUT_PROTECTION)
int flash_stm32_ex_op_rdp(const struct device *dev, const uintptr_t in,
			  void *out)
{
	const struct flash_stm32_ex_op_rdp *request =
		(const struct flash_stm32_ex_op_rdp *)in;
	struct flash_stm32_ex_op_rdp *result =
		(struct flash_stm32_ex_op_rdp *)out;

#ifdef CONFIG_USERSPACE
	struct flash_stm32_ex_op_rdp copy;
	bool syscall_trap = z_syscall_trap();
#endif
	int rc = 0, rc2 = 0;

	if (request != NULL) {
#ifdef CONFIG_USERSPACE
		if (syscall_trap) {
			Z_OOPS(z_user_from_copy(&copy, request, sizeof(copy)));
			request = &copy;
		}
#endif
		if (rc == 0) {
			rc = flash_stm32_update_rdp(dev, request->enable);
		}

	}

	if (result != NULL) {
#ifdef CONFIG_USERSPACE
		if (syscall_trap) {
			result = &copy;
		}
#endif
		rc2 = flash_stm32_get_rdp(dev, &result->enable);
		if (!rc) {
			rc = rc2;
		}

#ifdef CONFIG_USERSPACE
		if (syscall_trap) {
			Z_OOPS(z_user_to_copy(out, result, sizeof(copy)));
		}
#endif
	}

	return rc;
}
#endif /* CONFIG_FLASH_STM32_READOUT_PROTECTION */

static const struct flash_driver_api flash_stm32_api = {
	.erase = flash_stm32_erase,
	.write = flash_stm32_write,
	.read = flash_stm32_read,
	.get_parameters = flash_stm32_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_stm32_page_layout,
#endif
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_stm32_ex_op,
#endif
};

static int stm32_flash_init(const struct device *dev)
{
	int rc;
	/* Below is applicable to F0, F1, F3, G0, G4, L1, L4, L5, U5 & WB55 series.
	 * For F2, F4, F7 & H7 series, this is not applicable.
	 */
#if DT_INST_NODE_HAS_PROP(0, clocks)
	struct flash_stm32_priv *p = FLASH_STM32_PRIV(dev);
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/*
	 * On STM32 F0, F1, F3 & L1 series, flash interface clock source is
	 * always HSI, so statically enable HSI here.
	 */
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32F1X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L1X)
	LL_RCC_HSI_Enable();

	while (!LL_RCC_HSI_IsReady()) {
	}
#endif

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	if (clock_control_on(clk, (clock_control_subsys_t)&p->pclken) != 0) {
		LOG_ERR("Failed to enable clock");
		return -EIO;
	}
#endif

#ifdef CONFIG_SOC_SERIES_STM32WBX
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_HSEM);
#endif /* CONFIG_SOC_SERIES_STM32WBX */

	flash_stm32_sem_init(dev);

	LOG_DBG("Flash initialized. BS: %zu",
		flash_stm32_parameters.write_block_size);

	/* Check Flash configuration */
	rc = flash_stm32_check_configuration();
	if (rc < 0) {
		return rc;
	}

#if ((CONFIG_FLASH_LOG_LEVEL >= LOG_LEVEL_DBG) && CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout *layout;
	size_t layout_size;

	flash_stm32_page_layout(dev, &layout, &layout_size);
	for (size_t i = 0; i < layout_size; i++) {
		LOG_DBG("Block %zu: bs: %zu count: %zu", i,
			layout[i].pages_size, layout[i].pages_count);
	}
#endif

	return 0;
}

DEVICE_DT_INST_DEFINE(0, stm32_flash_init, NULL,
		    &flash_data, NULL, POST_KERNEL,
		    CONFIG_FLASH_INIT_PRIORITY, &flash_stm32_api);
