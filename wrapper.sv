`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
//
`define E203_PC_SIZE 32
`define E203_ADDR_SIZE 32
`define E203_INST_WIDTH 32
`define E203_XLEN 32
`define E203_HARTID_SIZE 4
`define E203_BOOTADDR_SIZE 32
`include "e203_defines.v"


module rvfi_wrapper (
	input         clock,
	input         reset,
	`RVFI_OUTPUTS
);

//----------------------------
// Signal Declarations
//----------------------------
logic [`E203_PC_SIZE-1:0] inspect_pc;
logic inspect_dbg_irq;
logic inspect_mem_cmd_valid;
logic inspect_mem_cmd_ready;
logic inspect_mem_rsp_valid;
logic inspect_mem_rsp_ready;
logic inspect_core_clk;

logic core_csr_clk;
logic core_wfi;
logic tm_stop;
logic [`E203_PC_SIZE-1:0] pc_rtvec;

logic dbg_irq_r;
logic [`E203_PC_SIZE-1:0] cmt_dpc;
logic cmt_dpc_ena;
logic [2:0] cmt_dcause;
logic cmt_dcause_ena;
logic wr_dcsr_ena;
logic wr_dpc_ena;
logic wr_dscratch_ena;
logic [31:0] wr_csr_nxt;
logic [31:0] dcsr_r;
logic [`E203_PC_SIZE-1:0] dpc_r;
logic [31:0] dscratch_r;

logic dbg_mode;
logic dbg_halt_r;
logic dbg_step_r;
logic dbg_ebreakm_r;
logic dbg_stopcycle;
logic dbg_irq_a;

logic [`E203_HART_ID_W-1:0] core_mhartid;
logic ext_irq_a;
logic sft_irq_a;
logic tmr_irq_a;

logic tcm_sd;
logic tcm_ds;

logic ppi_icb_cmd_valid;
logic ppi_icb_cmd_ready;
logic [`E203_ADDR_SIZE-1:0] ppi_icb_cmd_addr;
logic ppi_icb_cmd_read;
logic [`E203_XLEN-1:0] ppi_icb_cmd_wdata;
logic [`E203_XLEN/8-1:0] ppi_icb_cmd_wmask;
logic ppi_icb_rsp_valid;
logic ppi_icb_rsp_ready;
logic ppi_icb_rsp_err;
logic [`E203_XLEN-1:0] ppi_icb_rsp_rdata;

logic clint_icb_cmd_valid;
logic clint_icb_cmd_ready;
logic [`E203_ADDR_SIZE-1:0] clint_icb_cmd_addr;
logic clint_icb_cmd_read;
logic [`E203_XLEN-1:0] clint_icb_cmd_wdata;
logic [`E203_XLEN/8-1:0] clint_icb_cmd_wmask;
logic clint_icb_rsp_valid;
logic clint_icb_rsp_ready;
logic clint_icb_rsp_err;
logic [`E203_XLEN-1:0] clint_icb_rsp_rdata;

logic plic_icb_cmd_valid;
logic plic_icb_cmd_ready;
logic [`E203_ADDR_SIZE-1:0] plic_icb_cmd_addr;
logic plic_icb_cmd_read;
logic [`E203_XLEN-1:0] plic_icb_cmd_wdata;
logic [`E203_XLEN/8-1:0] plic_icb_cmd_wmask;
logic plic_icb_rsp_valid;
logic plic_icb_rsp_ready;
logic plic_icb_rsp_err;
logic [`E203_XLEN-1:0] plic_icb_rsp_rdata;

logic fio_icb_cmd_valid;
logic fio_icb_cmd_ready;
logic [`E203_ADDR_SIZE-1:0] fio_icb_cmd_addr;
logic fio_icb_cmd_read;
logic [`E203_XLEN-1:0] fio_icb_cmd_wdata;
logic [`E203_XLEN/8-1:0] fio_icb_cmd_wmask;
logic fio_icb_rsp_valid;
logic fio_icb_rsp_ready;
logic fio_icb_rsp_err;
logic [`E203_XLEN-1:0] fio_icb_rsp_rdata;
/*
logic mem_icb_cmd_valid;
logic mem_icb_cmd_ready;
logic [`E203_ADDR_SIZE-1:0] mem_icb_cmd_addr;
logic mem_icb_cmd_read;
logic [`E203_XLEN-1:0] mem_icb_cmd_wdata;
logic [`E203_XLEN/8-1:0] mem_icb_cmd_wmask;
logic mem_icb_rsp_valid;
logic mem_icb_rsp_ready;
logic mem_icb_rsp_err;
logic [`E203_XLEN-1:0] mem_icb_rsp_rdata;
*/
logic test_mode;
logic clk;
logic rst_n;


//----------------------------
// Module Instantiation
//----------------------------
e203_hbirdv2 u_e203_hbirdv2 (
  .inspect_pc(inspect_pc),
  .inspect_dbg_irq(inspect_dbg_irq),
  .inspect_mem_cmd_valid(inspect_mem_cmd_valid),
  .inspect_mem_cmd_ready(inspect_mem_cmd_ready),
  .inspect_mem_rsp_valid(inspect_mem_rsp_valid),
  .inspect_mem_rsp_ready(inspect_mem_rsp_ready),
  .inspect_core_clk(inspect_core_clk),

  .core_csr_clk(core_csr_clk),
  .core_wfi(core_wfi),
  .tm_stop(tm_stop),
  .pc_rtvec(pc_rtvec),

  .dbg_irq_r(dbg_irq_r),
  .cmt_dpc(cmt_dpc),
  .cmt_dpc_ena(cmt_dpc_ena),
  .cmt_dcause(cmt_dcause),
  .cmt_dcause_ena(cmt_dcause_ena),
  .wr_dcsr_ena(wr_dcsr_ena),
  .wr_dpc_ena(wr_dpc_ena),
  .wr_dscratch_ena(wr_dscratch_ena),
  .wr_csr_nxt(wr_csr_nxt),
  .dcsr_r(dcsr_r),
  .dpc_r(dpc_r),
  .dscratch_r(dscratch_r),

  .dbg_mode(dbg_mode),
  .dbg_halt_r(dbg_halt_r),
  .dbg_step_r(dbg_step_r),
  .dbg_ebreakm_r(dbg_ebreakm_r),
  .dbg_stopcycle(dbg_stopcycle),
  .dbg_irq_a(dbg_irq_a),

  .core_mhartid(core_mhartid),
  .ext_irq_a(ext_irq_a),
  .sft_irq_a(sft_irq_a),
  .tmr_irq_a(tmr_irq_a),
  .tcm_sd(tcm_sd),
  .tcm_ds(tcm_ds),

  .ppi_icb_cmd_valid(ppi_icb_cmd_valid),
  .ppi_icb_cmd_ready(ppi_icb_cmd_ready),
  .ppi_icb_cmd_addr(ppi_icb_cmd_addr),
  .ppi_icb_cmd_read(ppi_icb_cmd_read),
  .ppi_icb_cmd_wdata(ppi_icb_cmd_wdata),
  .ppi_icb_cmd_wmask(ppi_icb_cmd_wmask),
  .ppi_icb_rsp_valid(ppi_icb_rsp_valid),
  .ppi_icb_rsp_ready(ppi_icb_rsp_ready),
  .ppi_icb_rsp_err(ppi_icb_rsp_err),
  .ppi_icb_rsp_rdata(ppi_icb_rsp_rdata),

  .clint_icb_cmd_valid(clint_icb_cmd_valid),
  .clint_icb_cmd_ready(clint_icb_cmd_ready),
  .clint_icb_cmd_addr(clint_icb_cmd_addr),
  .clint_icb_cmd_read(clint_icb_cmd_read),
  .clint_icb_cmd_wdata(clint_icb_cmd_wdata),
  .clint_icb_cmd_wmask(clint_icb_cmd_wmask),
  .clint_icb_rsp_valid(clint_icb_rsp_valid),
  .clint_icb_rsp_ready(clint_icb_rsp_ready),
  .clint_icb_rsp_err(clint_icb_rsp_err),
  .clint_icb_rsp_rdata(clint_icb_rsp_rdata),

  .plic_icb_cmd_valid(plic_icb_cmd_valid),
  .plic_icb_cmd_ready(plic_icb_cmd_ready),
  .plic_icb_cmd_addr(plic_icb_cmd_addr),
  .plic_icb_cmd_read(plic_icb_cmd_read),
  .plic_icb_cmd_wdata(plic_icb_cmd_wdata),
  .plic_icb_cmd_wmask(plic_icb_cmd_wmask),
  .plic_icb_rsp_valid(plic_icb_rsp_valid),
  .plic_icb_rsp_ready(plic_icb_rsp_ready),
  .plic_icb_rsp_err(plic_icb_rsp_err),
  .plic_icb_rsp_rdata(plic_icb_rsp_rdata),

  .fio_icb_cmd_valid(fio_icb_cmd_valid),
  .fio_icb_cmd_ready(fio_icb_cmd_ready),
  .fio_icb_cmd_addr(fio_icb_cmd_addr),
  .fio_icb_cmd_read(fio_icb_cmd_read),
  .fio_icb_cmd_wdata(fio_icb_cmd_wdata),
  .fio_icb_cmd_wmask(fio_icb_cmd_wmask),
  .fio_icb_rsp_valid(fio_icb_rsp_valid),
  .fio_icb_rsp_ready(fio_icb_rsp_ready),
  .fio_icb_rsp_err(fio_icb_rsp_err),
  .fio_icb_rsp_rdata(fio_icb_rsp_rdata),

   /*
  .mem_icb_cmd_valid(mem_icb_cmd_valid),
  .mem_icb_cmd_ready(mem_icb_cmd_ready),
  .mem_icb_cmd_addr(mem_icb_cmd_addr),
  .mem_icb_cmd_read(mem_icb_cmd_read),
  .mem_icb_cmd_wdata(mem_icb_cmd_wdata),
  .mem_icb_cmd_wmask(mem_icb_cmd_wmask),
  .mem_icb_rsp_valid(mem_icb_rsp_valid),
  .mem_icb_rsp_ready(mem_icb_rsp_ready),
  .mem_icb_rsp_err(mem_icb_rsp_err),
  .mem_icb_rsp_rdata(mem_icb_rsp_rdata),
  */
  .test_mode(test_mode),
  .clk(clock),
  .rst_n(reset),

  `RVFI_CONN
);



endmodule