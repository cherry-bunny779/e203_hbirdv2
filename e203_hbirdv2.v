`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`include "e203_defines.v"

module e203_hbirdv2 (
	input         clock,
	input         reset,
	`RVFI_OUTPUTS
);

wrapper iDUT(.clock(clock),.reset(reset),`RVFI_CONN);

endmodule