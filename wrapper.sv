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

// === Signal Declarations ===
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

  logic mem_icb_cmd_valid;
  logic mem_icb_cmd_ready;
  logic [`E203_ADDR_SIZE-1:0] mem_icb_cmd_addr;
  logic mem_icb_cmd_read;
  logic [31:0] mem_icb_cmd_wdata;
  logic [3:0] mem_icb_cmd_wmask;
  logic mem_icb_cmd_burst;
  logic [1:0] mem_icb_cmd_beat;
  logic mem_icb_cmd_lock;
  logic mem_icb_cmd_excl;
  logic [1:0] mem_icb_cmd_size;

  logic mem_icb_rsp_valid;
  logic mem_icb_rsp_ready;
  logic [31:0] mem_icb_rsp_rdata;
  logic mem_icb_rsp_err;

  logic clk;
  logic rst_n;

  `ifdef E203_IRQ_NEED_SYNC
    logic ext_irq_async;
    logic sft_irq_async;
    logic tmr_irq_async;
  `else
    logic ext_irq;
    logic sft_irq;
    logic tmr_irq;
  `endif

  logic dbg_irq;
  logic [`E203_HARTID_SIZE-1:0] hartid;
  logic [`E203_BOOTADDR_SIZE-1:0] boot_addr;
  logic [`E203_PC_SIZE-1:0] mret_pc;

  `ifdef E203_HAS_ITCM
    logic itcm_ls;
    logic [`E203_ADDR_SIZE-1:0] itcm_addr;
    logic itcm_read;
    logic [31:0] itcm_wdata;
    logic [3:0] itcm_wmask;
    logic itcm_req;
    logic itcm_gnt;
    logic itcm_rvalid;
    logic [31:0] itcm_rdata;
  `endif

  `ifdef E203_HAS_DTCM
    logic dtcm_ls;
    logic [`E203_ADDR_SIZE-1:0] dtcm_addr;
    logic dtcm_read;
    logic [31:0] dtcm_wdata;
    logic [3:0] dtcm_wmask;
    logic dtcm_req;
    logic dtcm_gnt;
    logic dtcm_rvalid;
    logic [31:0] dtcm_rdata;
  `endif

  `ifdef E203_TIMING_ANALYSIS
    logic [`E203_PC_SIZE-1:0] commit_pc;
    logic commit_pc_vld;
    logic commit_inst_vld;
    logic [`E203_INST_WIDTH-1:0] commit_inst;
    logic commit_illegal;
    logic commit_exception;
    logic [4:0] commit_ecause;
    logic [`E203_XLEN-1:0] commit_mtval;
    logic commit_rdst_vld;
    logic [4:0] commit_rdst_idx;
    logic [`E203_XLEN-1:0] commit_rdst_dat;
    logic commit_bjp;
    logic commit_bjp_prdt;
    logic commit_bjp_rslv;
    logic commit_mret;
    logic commit_trap;
    logic wr_dtim;
    logic wr_itim;
    logic wr_sram;
    logic wr_clint;
    logic wr_other;
  `endif

  logic test_mode;

  // === DUT Instantiation ===
  e203_hbirdv2 iDUT (
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

    .mem_icb_cmd_valid(mem_icb_cmd_valid),
    .mem_icb_cmd_ready(mem_icb_cmd_ready),
    .mem_icb_cmd_addr(mem_icb_cmd_addr),
    .mem_icb_cmd_read(mem_icb_cmd_read),
    .mem_icb_cmd_wdata(mem_icb_cmd_wdata),
    .mem_icb_cmd_wmask(mem_icb_cmd_wmask),
    .mem_icb_cmd_burst(mem_icb_cmd_burst),
    .mem_icb_cmd_beat(mem_icb_cmd_beat),
    .mem_icb_cmd_lock(mem_icb_cmd_lock),
    .mem_icb_cmd_excl(mem_icb_cmd_excl),
    .mem_icb_cmd_size(mem_icb_cmd_size),

    .mem_icb_rsp_valid(mem_icb_rsp_valid),
    .mem_icb_rsp_ready(mem_icb_rsp_ready),
    .mem_icb_rsp_rdata(mem_icb_rsp_rdata),
    .mem_icb_rsp_err(mem_icb_rsp_err),

    `ifdef E203_HAS_ITCM
    .itcm_ls(itcm_ls),
    .itcm_addr(itcm_addr),
    .itcm_read(itcm_read),
    .itcm_wdata(itcm_wdata),
    .itcm_wmask(itcm_wmask),
    .itcm_req(itcm_req),
    .itcm_gnt(itcm_gnt),
    .itcm_rvalid(itcm_rvalid),
    .itcm_rdata(itcm_rdata),
    `endif

    `ifdef E203_HAS_DTCM
    .dtcm_ls(dtcm_ls),
    .dtcm_addr(dtcm_addr),
    .dtcm_read(dtcm_read),
    .dtcm_wdata(dtcm_wdata),
    .dtcm_wmask(dtcm_wmask),
    .dtcm_req(dtcm_req),
    .dtcm_gnt(dtcm_gnt),
    .dtcm_rvalid(dtcm_rvalid),
    .dtcm_rdata(dtcm_rdata),
    `endif

    .clk(clk),
    .rst_n(rst_n),

    `ifdef E203_IRQ_NEED_SYNC
    .ext_irq_async(ext_irq_async),
    .sft_irq_async(sft_irq_async),
    .tmr_irq_async(tmr_irq_async),
    `else
    .ext_irq(ext_irq),
    .sft_irq(sft_irq),
    .tmr_irq(tmr_irq),
    `endif

    .dbg_irq(dbg_irq),
    .hartid(hartid),
    .boot_addr(boot_addr),
    .mret_pc(mret_pc),

    `ifdef E203_TIMING_ANALYSIS
    .commit_pc(commit_pc),
    .commit_pc_vld(commit_pc_vld),
    .commit_inst_vld(commit_inst_vld),
    .commit_inst(commit_inst),
    .commit_illegal(commit_illegal),
    .commit_exception(commit_exception),
    .commit_ecause(commit_ecause),
    .commit_mtval(commit_mtval),
    .commit_rdst_vld(commit_rdst_vld),
    .commit_rdst_idx(commit_rdst_idx),
    .commit_rdst_dat(commit_rdst_dat),
    .commit_bjp(commit_bjp),
    .commit_bjp_prdt(commit_bjp_prdt),
    .commit_bjp_rslv(commit_bjp_rslv),
    .commit_mret(commit_mret),
    .commit_trap(commit_trap),
    .wr_dtim(wr_dtim),
    .wr_itim(wr_itim),
    .wr_sram(wr_sram),
    .wr_clint(wr_clint),
    .wr_other(wr_other),
    `endif

    .test_mode(test_mode)
  );


   // Implement RVFI
  /* rvfi_valid
    When the core retires an instruction, it asserts the rvfi_valid signal and 
    uses the signals described below to output the details of the retired instruction
  */
  // Two Stage Pipeline, IF || EXU.Decode EXU.Execute WB || Memory
  // if branch prediciton not wrong (IF) & PC_valid (EX), assume rvfi_valid
  // assign rvfi_valid = iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu_commit.alu_cmt_i_valid;
                      // &(!iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_ifu.ifu_halt_ack);
  logic [3:0] u_rvfi_mode;
  assign u_rvfi_mode[0] = iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.u_mode;
  assign u_rvfi_mode[1] = iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.s_mode;
  assign u_rvfi_mode[2] = iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.h_mode;
  assign u_rvfi_mode[3] = iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.m_mode;

  always_ff @(posedge clock ) begin : RVFI_Verify
    rvfi_valid <= reset && iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu_commit.alu_cmt_i_valid; // reset is active low
    rvfi_order <= reset ? rvfi_order + rvfi_valid : 0; // reset is active low

    rvfi_insn <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.ifu_o_ir;
    rvfi_trap <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.commit_trap;
    rvfi_halt <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.wfi_halt_ifu_req;
    rvfi_intr <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.u_e203_exu_commit.u_e203_exu_excp.irq_req_raw;
    rvfi_mode <= u_rvfi_mode;
    rvfi_ixl <= 1'b1; // XLEN=32
    rvfi_rs1_addr <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.ifu_o_rs1idx;
    rvfi_rs2_addr <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.ifu_o_rs2idx;
    rvfi_rs1_rdata <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.rf2ifu_rs1;
    rvfi_rs2_rdata <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.rf_rs2; // Not accessible at core level?
    rvfi_rd_addr <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.rf_wbck_rdidx;
    rvfi_rd_wdata <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.rf_wbck_wdat;
    
    rvfi_pc_rdata <= inspect_pc;
    rvfi_pc_wdata <= iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.alu_cmt_pc_vld ? 
                     iDUT.u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.alu_cmt_pc : rvfi_pc_wdata;
    /* Memory does not need to be tested
    rvfi_mem_addr <= 
    rvfi_mem_rmask <= 
    rvfi_mem_wmask <= 
    rvfi_mem_rdata <= 
    rvfi_mem_wdata <= 
    */
  end


endmodule