zephyr_include_directories(.)

zephyr_compile_options_ifndef(
  CONFIG_RISCV_GENERIC_TOOLCHAIN
  -march=imfcXpulpv2
  )

zephyr_sources(
  init.S
  vector.S
  soc_irq.S
  puppy_irq.c
  puppy_idle.c
  )

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
