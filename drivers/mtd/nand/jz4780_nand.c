/*
 * JZ4780 NAND driver
 *
 * Copyright (c) 2015 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/gpio/consumer.h>
#include <linux/of_mtd.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <linux/jz4780-nemc.h>

#include "jz4780_bch.h"

#define DRV_NAME	"jz4780-nand"

#define OFFSET_DATA	0x00000000
#define OFFSET_CMD	0x00400000
#define OFFSET_ADDR	0x00800000

/* Command delay when there is no R/B pin. */
#define RB_DELAY_US	100

struct jz4780_nand_cs {
	unsigned int bank;
	void __iomem *base;
};

struct jz4780_nand_controller {
	struct device *dev;
	struct device *bch;
	struct nand_hw_control controller;
	unsigned int num_banks;
	struct list_head chips;
	struct jz4780_nand_cs cs[];
};

struct jz4780_nand_chip {
	struct mtd_info mtd;
	struct nand_chip chip;
	struct list_head node;

	struct nand_ecclayout ecclayout;

	struct gpio_desc *busy_gpio;
	struct gpio_desc *wp_gpio;
	unsigned int busy_gpio_active_low: 1;
	unsigned int wp_gpio_active_low: 1;
	unsigned int reading: 1;

	int selected;
};

static inline struct jz4780_nand_chip *to_jz4780_nand_chip(struct mtd_info *mtd)
{
	return container_of(mtd, struct jz4780_nand_chip, mtd);
}

static inline struct jz4780_nand_controller *to_jz4780_nand_controller(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct jz4780_nand_controller, controller);
}

static void jz4780_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct jz4780_nand_chip *nand = to_jz4780_nand_chip(mtd);
	struct jz4780_nand_controller *nfc = to_jz4780_nand_controller(nand->chip.controller);
	struct jz4780_nand_cs *cs;

	if (chipnr == -1) {
		/* Ensure the currently selected chip is deasserted. */
		if (nand->selected >= 0) {
			cs = &nfc->cs[nand->selected];
			jz4780_nemc_assert(nfc->dev, cs->bank, false);
		}
	} else {
		cs = &nfc->cs[chipnr];
		nand->chip.IO_ADDR_R = cs->base + OFFSET_DATA;
		nand->chip.IO_ADDR_W = cs->base + OFFSET_DATA;
	}

	nand->selected = chipnr;
}

static void jz4780_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
				 unsigned int ctrl)
{
	struct jz4780_nand_chip *nand = to_jz4780_nand_chip(mtd);
	struct jz4780_nand_controller *nfc = to_jz4780_nand_controller(nand->chip.controller);
	struct jz4780_nand_cs *cs;

	if (WARN_ON(nand->selected < 0))
		return;

	cs = &nfc->cs[nand->selected];

	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_ALE)
			nand->chip.IO_ADDR_W = cs->base + OFFSET_ADDR;
		else if (ctrl & NAND_CLE)
			nand->chip.IO_ADDR_W = cs->base + OFFSET_CMD;
		else
			nand->chip.IO_ADDR_W = cs->base + OFFSET_DATA;
		jz4780_nemc_assert(nfc->dev, cs->bank, ctrl & NAND_NCE);
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, nand->chip.IO_ADDR_W);
}

static int jz4780_nand_dev_ready(struct mtd_info *mtd)
{
	struct jz4780_nand_chip *nand = to_jz4780_nand_chip(mtd);

	return !(gpiod_get_value_cansleep(nand->busy_gpio) ^ nand->busy_gpio_active_low);
}

static void jz4780_nand_ecc_hwctl(struct mtd_info *mtd, int mode)
{
	struct jz4780_nand_chip *nand = to_jz4780_nand_chip(mtd);

	nand->reading = (mode == NAND_ECC_READ);
}

static int jz4780_nand_ecc_calculate(struct mtd_info *mtd, const uint8_t *dat,
				     uint8_t *ecc_code)
{
	struct jz4780_nand_chip *nand = to_jz4780_nand_chip(mtd);
	struct jz4780_nand_controller *nfc = to_jz4780_nand_controller(nand->chip.controller);
	struct jz4780_bch_params params;

	/*
	 * Don't need to generate the ECC when reading, BCH does it for us as
	 * part of decoding/correction.
	 */
	if (nand->reading)
		return 0;

	params.size = nand->chip.ecc.size;
	params.bytes = nand->chip.ecc.bytes;
	params.strength = nand->chip.ecc.strength;

	return jz4780_bch_calculate(nfc->bch, &params, dat, ecc_code);
}

static int jz4780_nand_ecc_correct(struct mtd_info *mtd, uint8_t *dat,
				   uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct jz4780_nand_chip *nand = to_jz4780_nand_chip(mtd);
	struct jz4780_nand_controller *nfc = to_jz4780_nand_controller(nand->chip.controller);
	struct jz4780_bch_params params;

	params.size = nand->chip.ecc.size;
	params.bytes = nand->chip.ecc.bytes;
	params.strength = nand->chip.ecc.strength;

	return jz4780_bch_correct(nfc->bch, &params, dat, read_ecc);
}

static int jz4780_nand_init_ecc(struct jz4780_nand_chip *nand, struct device *dev)
{
	struct mtd_info *mtd = &nand->mtd;
	struct nand_chip *chip = &nand->chip;
	struct jz4780_nand_controller *nfc = to_jz4780_nand_controller(nand->chip.controller);
	struct nand_ecclayout *layout = &nand->ecclayout;
	struct device_node *bch_np;
	int ret = 0;
	uint32_t start, i;

	chip->ecc.bytes = fls(1 + 8 * chip->ecc.size) * chip->ecc.strength / 8;

	if (chip->ecc.mode == NAND_ECC_HW) {
		/* Only setup the BCH controller once. */
		if (!nfc->bch) {
			bch_np = of_parse_phandle(dev->of_node,
						"ingenic,bch-controller", 0);
			if (bch_np) {
				ret = jz4780_bch_get(bch_np, &nfc->bch);
				of_node_put(bch_np);
				if (ret)
					return ret;
			} else {
				dev_err(dev, "no bch controller in DT\n");
				return -ENODEV;
			}
		}

		chip->ecc.hwctl = jz4780_nand_ecc_hwctl;
		chip->ecc.calculate = jz4780_nand_ecc_calculate;
		chip->ecc.correct = jz4780_nand_ecc_correct;
	}

	if (chip->ecc.mode != NAND_ECC_NONE)
		dev_info(dev, "using %s BCH (strength %d, size %d, bytes %d)\n",
			(nfc->bch) ? "hardware" : "software", chip->ecc.strength,
			chip->ecc.size, chip->ecc.bytes);
	else
		dev_info(dev, "not using ECC\n");

	/* The NAND core will generate the ECC layout. */
	if (chip->ecc.mode == NAND_ECC_SOFT || chip->ecc.mode == NAND_ECC_SOFT_BCH)
		return 0;

	/* Generate ECC layout. ECC codes are right aligned in the OOB area. */
	layout->eccbytes = mtd->writesize / chip->ecc.size * chip->ecc.bytes;
	start = mtd->oobsize - layout->eccbytes;
	for (i = 0; i < layout->eccbytes; i++)
		layout->eccpos[i] = start + i;

	layout->oobfree[0].offset = 2;
	layout->oobfree[0].length = mtd->oobsize - layout->eccbytes - 2;

	chip->ecc.layout = layout;
	return 0;
}

static int jz4780_nand_init_chip(struct platform_device *pdev,
				struct jz4780_nand_controller *nfc,
				struct device_node *np,
				unsigned int chipnr)
{
	struct device *dev = &pdev->dev;
	struct jz4780_nand_chip *nand;
	struct jz4780_nand_cs *cs;
	struct resource *res;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct mtd_part_parser_data ppdata;
	int ret = 0;

	cs = &nfc->cs[chipnr];

	jz4780_nemc_set_type(nfc->dev, cs->bank,
				JZ4780_NEMC_BANK_NAND);

	res = platform_get_resource(pdev, IORESOURCE_MEM, chipnr);
	cs->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cs->base))
		return PTR_ERR(cs->base);

	nand = devm_kzalloc(dev, sizeof(*nand), GFP_KERNEL);
	if (!nand)
		return -ENOMEM;

	nand->busy_gpio = devm_gpiod_get_optional(dev, "rb", GPIOD_IN);

	if (IS_ERR(nand->busy_gpio)) {
		ret = PTR_ERR(nand->busy_gpio);
		dev_err(dev, "failed to request busy GPIO: %d\n", ret);
		return ret;
	} else if (nand->busy_gpio) {
		nand->busy_gpio_active_low = gpiod_is_active_low(nand->busy_gpio);
		nand->chip.dev_ready = jz4780_nand_dev_ready;
	}

	nand->wp_gpio = devm_gpiod_get_optional(dev, "wp", GPIOD_OUT_LOW);

	if (IS_ERR(nand->wp_gpio)) {
		ret = PTR_ERR(nand->wp_gpio);
		dev_err(dev, "failed to request WP GPIO: %d\n", ret);
		return ret;
	} else if (nand->wp_gpio) {
		nand->wp_gpio_active_low = gpiod_is_active_low(nand->wp_gpio);
	}

	nand->selected = -1;
	mtd = &nand->mtd;
	chip = &nand->chip;
	mtd->priv = chip;
	mtd->owner = THIS_MODULE;
	mtd->name = DRV_NAME;
	mtd->dev.parent = dev;

	chip->flash_node = np;
	chip->chip_delay = RB_DELAY_US;
	chip->options = NAND_NO_SUBPAGE_WRITE;
	chip->select_chip = jz4780_nand_select_chip;
	chip->cmd_ctrl = jz4780_nand_cmd_ctrl;
	chip->ecc.mode = NAND_ECC_NONE;
	chip->controller = &nfc->controller;

	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret)
		return ret;

	ret = jz4780_nand_init_ecc(nand, dev);
	if (ret)
		return ret;

	ret = nand_scan_tail(mtd);
	if (ret)
		goto err_release_bch;

	ppdata.of_node = np;
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
	if (ret)
		goto err_release_nand;

	return 0;

err_release_nand:
	nand_release(mtd);

err_release_bch:
	if (nfc->bch)
		jz4780_bch_release(nfc->bch);

	return ret;
}

static int jz4780_nand_init_chips(struct jz4780_nand_controller *nfc,
				  struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct jz4780_nand_cs *cs;
	int i = 0;
	int ret;
	const __be32 *reg;
	int num_chips = of_get_child_count(dev->of_node);

	if (num_chips > nfc->num_banks) {
		dev_err(dev, "found %d chips but only %d banks\n", num_chips, nfc->num_banks);
		return -EINVAL;
	}


	/*
	 * Iterate over each bank assigned to this device and request resources.
	 * The bank numbers may not be consecutive, but nand_scan_ident()
	 * expects chip numbers to be, so fill out a consecutive array of chips
	 * which map chip number to actual bank number.
	 */
	for_each_child_of_node(dev->of_node, np) {
		/* TODO: Maybe move this into init_chip. */
		cs = &nfc->cs[i];

		reg = of_get_property(np, "reg", NULL);
		if (reg == NULL)
			return -EINVAL;

		cs->bank = be32_to_cpu(*reg);

		ret = jz4780_nand_init_chip(pdev, nfc, np, i);
		if (ret)
			return ret;

		i++;
	}

	return 0;
}


static int jz4780_nand_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned int num_banks;
	struct jz4780_nand_controller *nfc;
	int ret;

	num_banks = jz4780_nemc_num_banks(dev);
	if (num_banks == 0) {
		dev_err(dev, "no banks found\n");
		return -ENODEV;
	}

	nfc = devm_kzalloc(dev, sizeof(*nfc) + (sizeof(nfc->cs[0]) * num_banks), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nfc->dev = dev;
	nfc->num_banks = num_banks;

	spin_lock_init(&nfc->controller.lock);
	INIT_LIST_HEAD(&nfc->chips);
	init_waitqueue_head(&nfc->controller.wq);

	ret = jz4780_nand_init_chips(nfc, pdev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, nfc);
	return 0;
}

static int jz4780_nand_remove(struct platform_device *pdev)
{
	struct jz4780_nand_chip *nand = platform_get_drvdata(pdev);
	struct jz4780_nand_controller *nfc = to_jz4780_nand_controller(nand->chip.controller);

	if (nfc->bch)
		jz4780_bch_release(nfc->bch);

	return 0;
}

static const struct of_device_id jz4780_nand_dt_match[] = {
	{ .compatible = "ingenic,jz4780-nand" },
	{},
};
MODULE_DEVICE_TABLE(of, jz4780_nand_dt_match);

static struct platform_driver jz4780_nand_driver = {
	.probe		= jz4780_nand_probe,
	.remove		= jz4780_nand_remove,
	.driver	= {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(jz4780_nand_dt_match),
	},
};
module_platform_driver(jz4780_nand_driver);

MODULE_AUTHOR("Alex Smith <alex.smith@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 NAND driver");
MODULE_LICENSE("GPL v2");
