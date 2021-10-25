/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * General info from:
 * ARM CoreSight Architecture Specification v3.0 IHI0029E
 */

#ifndef OPENOCD_TARGET_ARM_CORESIGHT_H
#define OPENOCD_TARGET_ARM_CORESIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include <helper/bits.h>

#define ARM_CS_ALIGN                            (0x1000)

/* mandatory registers */
#define ARM_CS_PIDR0                            (0xFE0)
#define ARM_CS_PIDR1                            (0xFE4)
#define ARM_CS_PIDR2                            (0xFE8)
#define ARM_CS_PIDR3                            (0xFEC)
#define ARM_CS_PIDR4                            (0xFD0)
#define ARM_CS_PIDR5                            (0xFD4)
#define ARM_CS_PIDR6                            (0xFD8)
#define ARM_CS_PIDR7                            (0xFDC)

/*
 * When PIDR bit JEDEC is zero, only the lowers 7 bits of DESIGNER are valid
 * and represent a legacy ASCII Identity Code.
 */
#define ARM_CS_PIDR_PART(pidr)                  ((pidr) & 0x0FFF)
#define ARM_CS_PIDR_DESIGNER(pidr)              \
({                                              \
	typeof(pidr) _x = (pidr);                   \
	((_x >> 25) & 0x780) | ((_x >> 12) & 0x7F); \
})
#define ARM_CS_PIDR_JEDEC                       BIT(19)
#define ARM_CS_PIDR_SIZE(pidr)                  (((pidr) >> 36) & 0x000F)

#define ARM_CS_CIDR0                            (0xFF0)
#define ARM_CS_CIDR1                            (0xFF4)
#define ARM_CS_CIDR2                            (0xFF8)
#define ARM_CS_CIDR3                            (0xFFC)

#define ARM_CS_CIDR_CLASS_MASK                  (0x0000F000)
#define ARM_CS_CIDR_CLASS_SHIFT                 (12)
#define ARM_CS_CLASS_0X1_ROM_TABLE              (0x1)
#define ARM_CS_CLASS_0X9_CS_COMPONENT           (0x9)

#define ARM_CS_CIDR1_CLASS_MASK                 (0x000000F0)
#define ARM_CS_CIDR1_CLASS_SHIFT                (4)

static inline bool is_valid_arm_cs_cidr(uint32_t cidr)
{
	return (cidr & ~ARM_CS_CIDR_CLASS_MASK) == 0xB105000D;
}

/* Class 0x9 only registers */
#define ARM_CS_C9_DEVARCH                       (0xFBC)

#define ARM_CS_C9_DEVARCH_ARCHID_MASK           (0x0000FFFF)
#define ARM_CS_C9_DEVARCH_ARCHID_SHIFT          (0)
#define ARM_CS_C9_DEVARCH_REVISION_MASK         (0x000F0000)
#define ARM_CS_C9_DEVARCH_REVISION_SHIFT        (16)
#define ARM_CS_C9_DEVARCH_PRESENT               BIT(20)
#define ARM_CS_C9_DEVARCH_ARCHITECT_MASK        (0xFFE00000)
#define ARM_CS_C9_DEVARCH_ARCHITECT_SHIFT       (21)

#define ARM_CS_C9_DEVID                         (0xFC8)

#define ARM_CS_C9_DEVID_FORMAT_MASK             (0x0000000F)
#define ARM_CS_C9_DEVID_FORMAT_32BIT            (0)
#define ARM_CS_C9_DEVID_FORMAT_64BIT            (1)
#define ARM_CS_C9_DEVID_SYSMEM_MASK             BIT(4)
#define ARM_CS_C9_DEVID_PRR_MASK                BIT(5)
#define ARM_CS_C9_DEVID_CP_MASK                 BIT(5)

#define ARM_CS_C9_DEVTYPE                       (0xFCC)

#define ARM_CS_C9_DEVTYPE_MAJOR_MASK            (0x0000000F)
#define ARM_CS_C9_DEVTYPE_MAJOR_SHIFT           (0)
#define ARM_CS_C9_DEVTYPE_SUB_MASK              (0x000000F0)
#define ARM_CS_C9_DEVTYPE_SUB_SHIFT             (4)

#define ARM_CS_C9_DEVTYPE_MASK                  (0x000000FF)
#define ARM_CS_C9_DEVTYPE_CORE_DEBUG            (0x00000015)

/* Class 0x1 only registers */
#define ARM_CS_C1_MEMTYPE                       ARM_CS_C9_DEVTYPE

#define ARM_CS_C1_MEMTYPE_SYSMEM_MASK           BIT(0)

/* The coding of ROM entry present differs between Class 0x9 and Class 0x1,
 * but we can simplify the whole management */
#define ARM_CS_ROMENTRY_PRESENT                 BIT(0)
#define ARM_CS_ROMENTRY_OFFSET_MASK             (0xFFFFF000U)

#endif /* OPENOCD_TARGET_ARM_CORESIGHT_H */
