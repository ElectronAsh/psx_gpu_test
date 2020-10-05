# Test MiSTer core for Laxer's PSX GPU core.

I used the latest SNES core as the basis for this, but added the modified sysmem.sv, to allow the use of the AXI bridge.

The axi_test repo has some C code for sending example commands to the GPU core from HPS to the FPGA.
