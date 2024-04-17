# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F030R8" "--speed=4000")
board_runner_args(pyocd "--target=stm32f030cctx")
board_runner_args(probe-rs "--chip=STM32F030CCTx")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/probe-rs.board.cmake)
