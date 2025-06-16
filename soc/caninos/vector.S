#include <zephyr/toolchain.h>

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__puppy_init)
GTEXT(_isr_wrapper)

	.org 0x00
SECTION_FUNC(vectors, ivt)
	.option norvc;

	/*.rept 32*/
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	jal x0, _isr_wrapper
	/*.endr*/

	/* Call __reset for reset vector */
	jal x0, __start
	/* Illegal instruction */
	jal x0, _isr_wrapper
	/* ECALL (system call) */
	jal x0, _isr_wrapper
	/* Invalid memory access */
	jal x0, _isr_wrapper

SECTION_FUNC(vectors, __start)
	/* Set mtvec to point at ivt. */
	la t0, ivt
	csrw 0x305, t0
	/* Call into Zephyr initialization. */
	tail __puppy_init
