#include <zephyr/toolchain.h>

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__puppy_init)
GTEXT(_isr_wrapper)

.option norvc;

	.org 0x00
SECTION_FUNC(vectors, __start)
	/* Set mtvec to point at ivt. */
	la t0, ivt
	csrw mtvec, t0
	/* Call into Zephyr initialization. */
	tail __puppy_init

SECTION_FUNC(vectors, ivt)
	.rept 32
	jal zero, _isr_wrapper
	.endr
	/* Call __reset for reset vector */
	jal zero, __start
	/* Illegal instruction */
	jal zero, _isr_wrapper
	/* ECALL (system call) */
	jal zero, _isr_wrapper
	/* Invalid memory access */
	jal zero, _isr_wrapper

