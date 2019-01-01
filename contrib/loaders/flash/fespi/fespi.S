#define SPIFLASH_READ_STATUS	0x05 // Read Status Register
#define SPIFLASH_BSY_BIT		0x00000001 // WIP Bit of SPI SR on SMI SR

// Register offsets
#define FESPI_REG_FMT             0x40
#define FESPI_REG_TXFIFO          0x48
#define FESPI_REG_RXFIFO          0x4c
#define FESPI_REG_IP              0x74

// Fields
#define FESPI_IP_TXWM             0x1
#define FESPI_FMT_DIR(x)          (((x) & 0x1) << 3)

// To enter, jump to the start of command_table (ie. offset 0).
//      a0 - FESPI base address
//      a1 - start address of buffer

// The buffer contains a "program" in byte sequences. The first byte in a
// sequence determines the operation. Some operation will read more data from
// the program, while some will not. The operation byte is the offset into
// command_table, so eg. 4 means exit, 8 means transmit, and so on.

		.global _start
_start:
command_table:
		j       main            // 0
		ebreak                  // 4
		j       tx              // 8
		j       txwm_wait       // 12
		j       write_reg       // 16
		j		wip_wait		// 20
		j		set_dir			// 24

// Execute the program.
main:
		lbu     t0, 0(a1)
		addi    a1, a1, 1
		la      t1, command_table
		add     t0, t0, t1
		jr      t0

// Read 1 byte the contains the number of bytes to transmit. Then read those
// bytes from the program and transmit them one by one.
tx:
		lbu     t1, 0(a1)       // read number of bytes to transmit
		addi    a1, a1, 1
1:      lw      t0, FESPI_REG_TXFIFO(a0)        // wait for FIFO clear
		bltz    t0, 1b
		lbu     t0, 0(a1)       // Load byte to write
		sw      t0, FESPI_REG_TXFIFO(a0)
		addi    a1, a1, 1
		addi    t1, t1, -1
		bgtz    t1, 1b
		j       main

// Wait until TXWM is set.
txwm_wait:
1:      lw      t0, FESPI_REG_IP(a0)
		andi    t0, t0, FESPI_IP_TXWM
		beqz    t0, 1b
		j       main

// Read 1 byte that contains the offset of the register to write, and 1 byte
// that contains the data to write.
write_reg:
		lbu     t0, 0(a1)       // read register to write
		add     t0, t0, a0
		lbu     t1, 1(a1)       // read value to write
		addi    a1, a1, 2
		sw      t1, 0(t0)
		j       main

wip_wait:
		li		a2, SPIFLASH_READ_STATUS
		jal		txrx_byte
		// discard first result
1:		li		a2, 0
		jal		txrx_byte
		andi	t0, a2, SPIFLASH_BSY_BIT
		bnez	t0, 1b
		j		main

txrx_byte:	// transmit the byte in a2, receive a bit into a2
		lw      t0, FESPI_REG_TXFIFO(a0)        // wait for FIFO clear
		bltz    t0, txrx_byte
		sw      a2, FESPI_REG_TXFIFO(a0)
1:		lw		a2, FESPI_REG_RXFIFO(a0)
		bltz	a2, 1b
		ret

set_dir:
		lw		t0, FESPI_REG_FMT(a0)
		li		t1, ~(FESPI_FMT_DIR(0xFFFFFFFF))
		and		t0, t0, t1
		lbu     t1, 0(a1)       // read value to OR in
		addi    a1, a1, 1
		or		t0, t0, t1
		sw		t0, FESPI_REG_FMT(a0)
		j		main
