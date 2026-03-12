import cocotb
from cocotb.triggers import Timer

@cocotb.test()
async def basic_test(dut):
    # apply some example inputs
    dut.ui_in.value = 0
    dut.uio_in.value = 0

    # wait a little while
    await Timer(1, units="us")

    # just print the outputs
    dut._log.info(f"uo_out = {dut.uo_out.value}")