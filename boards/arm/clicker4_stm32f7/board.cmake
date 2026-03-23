board_runner_args(pyocd
  "--target=stm32f745vg"
  "--frequency=10000"
  "--flash-opt=-O connect_mode=under-reset"
)

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

# Use CMSIS-DAP + STM32F7 target defaults for CodeGrip/OpenOCD.
board_runner_args(openocd
  "--config=interface/cmsis-dap.cfg"
  "--config=target/stm32f7x.cfg"
  "--cmd-pre-init=cmsis_dap_backend hid"
  "--cmd-pre-init=adapter speed 400"
  "--cmd-reset-halt=reset init"
)

# Make OpenOCD the default runner for flash/debug (pyOCD can still be selected explicitly).
board_set_flasher(openocd)
board_set_debugger(openocd)
