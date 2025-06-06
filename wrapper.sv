`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`include "e203_defines.v"

module rvfi_wrapper (
	input         clock,
	input         reset,
	`RVFI_OUTPUTS
);
	(* keep *) wire trap;

	(* keep *) `rvformal_rand_reg mem_ready;
	(* keep *) `rvformal_rand_reg [31:0] mem_rdata;

	(* keep *) wire        mem_valid;
	(* keep *) wire        mem_instr;
	(* keep *) wire [31:0] mem_addr;
	(* keep *) wire [31:0] mem_wdata;
	(* keep *) wire [3:0]  mem_wstrb;

/* [To-Do] Need to incorporate RVFI_CONN32 in 
   riscv-formal-main/checks/rvfi_macros.vh 
*/
// MACROS defined in ./e203_defines.v default is 32 in ./config.v
/* LIST OF MACROS [To-Do]
   E203_PC_SIZE = 32 (default)
   E203_HART_ID_W = 1 (?)
   E203_HAS_ITCM_EXTITF (--> ifdef E203_CFG_HAS_ITCM --> ./config.v (default yes))
   E203_HAS_DTCM_EXTITF (--> ifdef E203_CFG_HAS_DTCM --> ./config.v (default yes))
   E203_ITCM_ADDR_WIDTH = 16
   E203_DTCM_ADDR_WIDTH  = 16 ( ./e203_defines.v --> E203_CFG_DTCM_ADDR_WIDTH --> ./config.v)
   E203_XLEN  = 32 (./e203_defines.v --> ifdef E203_CFG_XLEN_IS_32 --> e203_defines.v (default yes))
   E203_ADDR_SIZE = 32 (./config.v)
*/

// Before debug output logic
  logic [`E203_PC_SIZE-1:0] inspect_pc;
  logic inspect_dbg_irq,inspect_mem_cmd_valid,inspect_mem_cmd_ready,inspect_mem_rsp_valid,
      inspect_mem_rsp_ready,inspect_core_clk,core_csr_clk,core_wfi,tm_stop;
// Before debug input logic
  logic [`E203_PC_SIZE-1:0] pc_rtvec;

// Debug Module in
  logic [`E203_PC_SIZE-1:0] dpc_r;
  logic [32-1:0] dcsr_r,dscratch_r;
  logic dbg_mode,dbg_halt_r,dbg_step_r,dbg_ebreakm_r,dbg_stopcycle,dbg_irq_a;
// Debug Module out
  logic [`E203_PC_SIZE-1:0] cmt_dpc;
  logic [32-1:0] wr_csr_nxt;
  logic [3-1:0] cmt_dcause; // [3-1] ??
  logic dbg_irq_r,cmt_dpc_ena,cmt_dcause_ena,wr_dcsr_ena,wr_dpc_ena,wr_dscratch_ena;

// interrupt and other inputs
  logic [`E203_HART_ID_W-1:0] core_mhartid; 
  logic ext_irq_a,sft_irq_a,tmr_irq_a,tcm_sd,tcm_ds,ext2itcm_icb_rsp_ready;

// ITCM - IN
  logic [`E203_ITCM_ADDR_WIDTH-1:0] ext2itcm_icb_cmd_addr;
  logic [`E203_XLEN-1:0] ext2itcm_icb_cmd_wdata;
  logic [`E203_XLEN/8-1:0] ext2itcm_icb_cmd_wmask;
  logic ext2itcm_icb_cmd_valid,ext2itcm_icb_cmd_read;
// ITCM - OUT
  logic [`E203_XLEN-1:0] ext2itcm_icb_rsp_rdata;
  logic ext2itcm_icb_cmd_ready, ext2itcm_icb_rsp_valid,ext2itcm_icb_rsp_err,ext2itcm_icb_rsp_rdata;

// DTCM - IN
  logic  [`E203_DTCM_ADDR_WIDTH-1:0]   ext2dtcm_icb_cmd_addr;
  logic  [`E203_XLEN-1:0]        ext2dtcm_icb_cmd_wdata;
  logic  [`E203_XLEN/8-1:0]      ext2dtcm_icb_cmd_wmask;
  logic ext2dtcm_icb_cmd_valid,ext2dtcm_icb_cmd_wmask,ext2dtcm_icb_cmd_read,ext2dtcm_icb_rsp_ready;
// DTCM - OUT
  logic [`E203_XLEN-1:0]        ext2dtcm_icb_rsp_rdata;
  logic ext2dtcm_icb_cmd_ready,ext2dtcm_icb_rsp_valid,ext2dtcm_icb_rsp_err;

// Private Peripheral Interface - IN
  logic  [`E203_XLEN-1:0] ppi_icb_rsp_rdata;
  logic ppi_icb_cmd_ready,ppi_icb_rsp_valid,ppi_icb_rsp_err;
// // Private Peripheral Interface - OUT
  logic [`E203_ADDR_SIZE-1:0]   ppi_icb_cmd_addr; 
  logic [`E203_XLEN-1:0]        ppi_icb_cmd_wdata;
  logic [`E203_XLEN/8-1:0]      ppi_icb_cmd_wmask;
  logic ppi_icb_cmd_valid,ppi_icb_cmd_read,ppi_icb_rsp_ready;

// CLINT (ICB) - IN
  logic clint_icb_cmd_ready,clint_icb_rsp_valid,clint_icb_rsp_err;
  logic  [`E203_XLEN-1:0]       clint_icb_rsp_rdata;
// CLINT (ICB) - OUT
  logic clint_icb_cmd_valid,clint_icb_cmd_read,clint_icb_rsp_ready;
  logic [`E203_ADDR_SIZE-1:0]   clint_icb_cmd_addr;
  logic [`E203_XLEN-1:0]        clint_icb_cmd_wdata;
  logic [`E203_XLEN/8-1:0]      clint_icb_cmd_wmask;

// PLIC Interface (ICB) - IN
  logic plic_icb_cmd_ready, plic_icb_rsp_valid,plic_icb_rsp_err;
  logic  [`E203_XLEN-1:0]       plic_icb_rsp_rdata;
// PLIC Interface (ICB) - OUT
  logic plic_icb_cmd_valid,plic_icb_rsp_ready,plic_icb_cmd_read;
  logic [`E203_ADDR_SIZE-1:0]   plic_icb_cmd_addr;
  logic [`E203_XLEN-1:0]        plic_icb_cmd_wdata;
  logic [`E203_XLEN/8-1:0]      plic_icb_cmd_wmask;

// Fast IO Interface (ICB) - IN
  logic fio_icb_cmd_valid,fio_icb_cmd_ready, fio_icb_rsp_valid,fio_icb_rsp_err;
  logic  [`E203_XLEN-1:0]        fio_icb_rsp_rdata;
// Fast IO Interface (ICB) - OUT
  logic fio_icb_cmd_valid,fio_icb_cmd_read,fio_icb_rsp_ready;
  logic [`E203_ADDR_SIZE-1:0]   fio_icb_cmd_addr;
  logic [`E203_XLEN-1:0]        fio_icb_cmd_wdata;
  logic [`E203_XLEN/8-1:0]      fio_icb_cmd_wmask;

// System Memory Interface (ICB) - IN
  logic mem_icb_cmd_ready,mem_icb_rsp_valid,mem_icb_rsp_err;
  logic  [`E203_XLEN-1:0]        mem_icb_rsp_rdata;
// System Memory Interface (ICB) - OUT
  logic mem_icb_cmd_valid,mem_icb_cmd_read,mem_icb_rsp_ready;
  logic [`E203_ADDR_SIZE-1:0]   mem_icb_cmd_addr;
  logic [`E203_XLEN-1:0]        mem_icb_cmd_wdata;
  logic [`E203_XLEN/8-1:0]      mem_icb_cmd_wmask;

// Other signals
logic test_mode;

e203_hbirdv2 iDUT(
  .inspect_pc(inspect_pc),
  .inspect_dbg_irq(inspect_dbg_irq),
  .inspect_mem_cmd_valid(inspect_mem_cmd_valid),
  .inspect_mem_cmd_ready(inspect_mem_cmd_ready),
  .inspect_mem_rsp_valid(inspect_mem_rsp_valid),
  .inspect_mem_rsp_ready(inspect_mem_rsp_ready),
  .inspect_core_clk(inspect_core_clk),

  .core_csr_clk(core_csr_clk),

    

    // If this signal is high, then indicate the Core have executed WFI instruction
    //   and entered into the sleep state
  .core_wfi(core_wfi),

    // This signal is from our self-defined COUNTERSTOP (0xBFF) CSR's TM field
    //   software can programe this CSR to turn off the MTIME timer to save power
    // If this signal is high, then the MTIME timer from CLINT module will stop counting
  .tm_stop(tm_stop),

    // This signal can be used to indicate the PC value for the core after reset
  .pc_rtvec(pc_rtvec),

  ///////////////////////////////////////
  // The interface to Debug Module: Begin
  //
    // The synced debug interrupt back to Debug module 
  .dbg_irq_r(dbg_irq_r),

    // The debug mode CSR registers control interface from/to Debug module
  .cmt_dpc(cmt_dpc),
  .cmt_dpc_ena(cmt_dpc_ena),
  .cmt_dcause(cmt_dcause),
  .cmt_dcause_ena(cmt_dcause_ena),
  .wr_dcsr_ena(wr_dcsr_ena),
  .wr_dpc_ena(wr_dpc_ena),
  .wr_dscratch_ena(wr_dscratch_ena),
  .wr_csr_nxt(wr_csr_nxt),
  .dcsr_r(dcsr_r),  // [To-Do]
  .dpc_r(dpc_r),  // [To-Do]
  .dscratch_r(dscratch_r), //[To-Do]

    // The debug mode control signals from Debug Module -- All Below [To-Do]
  .dbg_mode(dbg_mode),
  .dbg_halt_r(dbg_halt_r),
  .dbg_step_r(dbg_step_r),
  .dbg_ebreakm_r(dbg_ebreakm_r),
  .dbg_stopcycle(dbg_stopcycle),
  .dbg_irq_a(dbg_irq_a),
  // The interface to Debug Module: End


    // This signal can be used to indicate the HART ID for this core
  .core_mhartid(core_mhartid),  

    // The External Interrupt signal from PLIC
  .ext_irq_a(ext_irq_a),
    // The Software Interrupt signal from CLINT
  .sft_irq_a(sft_irq_a),
    // The Timer Interrupt signal from CLINT
  .tmr_irq_a(tmr_irq_a),
  
  
    // The PMU control signal from PMU to control the TCM Shutdown
  .tcm_sd(tcm_sd),
    // The PMU control signal from PMU to control the TCM Deep-Sleep
  .tcm_ds(tcm_ds),
    
  `ifdef E203_HAS_ITCM_EXTITF //{ Instruction Tightly-Coupled Memory
  //////////////////////////////////////////////////////////////
  // External interface (ICB) to access ITCM: Begin
  //    * Bus cmd channel
  .ext2itcm_icb_cmd_valid(ext2itcm_icb_cmd_valid),
  .ext2itcm_icb_cmd_ready(ext2itcm_icb_cmd_ready),
  .ext2itcm_icb_cmd_addr(ext2itcm_icb_cmd_addr), 
  .ext2itcm_icb_cmd_read(ext2itcm_icb_cmd_read), 
  .ext2itcm_icb_cmd_wdata(ext2itcm_icb_cmd_wdata),
  .ext2itcm_icb_cmd_wmask(ext2itcm_icb_cmd_wmask),
  //
  //    * Bus RSP channel
  .ext2itcm_icb_rsp_valid(ext2itcm_icb_rsp_valid),
  .ext2itcm_icb_rsp_ready(ext2itcm_icb_rsp_ready),
  .ext2itcm_icb_rsp_err(ext2itcm_icb_rsp_err),
  .ext2itcm_icb_rsp_rdata(ext2itcm_icb_rsp_rdata),

  // External interface (ICB) to access ITCM: End
  `endif//}

  `ifdef E203_HAS_DTCM_EXTITF //{ Data Tightly Coupled Memory
  //////////////////////////////////////////////////////////////
  // External interface (ICB) to access DTCM: Start
  //    * Bus cmd channel
  .ext2dtcm_icb_cmd_valid(ext2dtcm_icb_cmd_valid),
  .ext2dtcm_icb_cmd_ready(ext2dtcm_icb_cmd_ready),
  .ext2dtcm_icb_cmd_addr(ext2dtcm_icb_cmd_addr), 
  .ext2dtcm_icb_cmd_read(ext2dtcm_icb_cmd_read), 
  .ext2dtcm_icb_cmd_wdata(ext2dtcm_icb_cmd_wdata),
  .ext2dtcm_icb_cmd_wmask(ext2dtcm_icb_cmd_wmask),
  //
  //    * Bus RSP channel
  .ext2dtcm_icb_rsp_valid(ext2dtcm_icb_rsp_valid),
  .ext2dtcm_icb_rsp_ready(ext2dtcm_icb_rsp_ready),
  .ext2dtcm_icb_rsp_err(ext2dtcm_icb_rsp_err),
  .ext2dtcm_icb_rsp_rdata(ext2dtcm_icb_rsp_rdata),
  // External interface (ICB) to access DTCM: End
  `endif//}

  
  //////////////////////////////////////////////////////////////
  // The Private Peripheral Interface (ICB): Begin
  //
  //    * Bus cmd channel
    .ppi_icb_cmd_valid     (ppi_icb_cmd_valid),
    .ppi_icb_cmd_ready     (ppi_icb_cmd_ready),
    .ppi_icb_cmd_addr      (ppi_icb_cmd_addr ),
    .ppi_icb_cmd_read      (ppi_icb_cmd_read ),
    .ppi_icb_cmd_wdata     (ppi_icb_cmd_wdata),
    .ppi_icb_cmd_wmask     (ppi_icb_cmd_wmask),
  //
  //    * Bus RSP channel
    .ppi_icb_rsp_valid     (ppi_icb_rsp_valid),
    .ppi_icb_rsp_ready     (ppi_icb_rsp_ready),
    .ppi_icb_rsp_err       (ppi_icb_rsp_err  ),
    .ppi_icb_rsp_rdata     (ppi_icb_rsp_rdata),
  // The Private Peripheral Interface (ICB): End

  //////////////////////////////////////////////////////////////
  // The CLINT Interface (ICB): Begin
    .clint_icb_cmd_valid     (clint_icb_cmd_valid),
    .clint_icb_cmd_ready     (clint_icb_cmd_ready),
    .clint_icb_cmd_addr      (clint_icb_cmd_addr ),
    .clint_icb_cmd_read      (clint_icb_cmd_read ),
    .clint_icb_cmd_wdata     (clint_icb_cmd_wdata),
    .clint_icb_cmd_wmask     (clint_icb_cmd_wmask),
    
    .clint_icb_rsp_valid     (clint_icb_rsp_valid),
    .clint_icb_rsp_ready     (clint_icb_rsp_ready),
    .clint_icb_rsp_err       (clint_icb_rsp_err  ),
    .clint_icb_rsp_rdata     (clint_icb_rsp_rdata),
  // The CLINT Interface (ICB): End

  //////////////////////////////////////////////////////////////
  // The PLIC Interface (ICB): Begin
    .plic_icb_cmd_valid     (plic_icb_cmd_valid),
    .plic_icb_cmd_ready     (plic_icb_cmd_ready),
    .plic_icb_cmd_addr      (plic_icb_cmd_addr ),
    .plic_icb_cmd_read      (plic_icb_cmd_read ),
    .plic_icb_cmd_wdata     (plic_icb_cmd_wdata),
    .plic_icb_cmd_wmask     (plic_icb_cmd_wmask),
  //    * Bus RSP channel
    .plic_icb_rsp_valid     (plic_icb_rsp_valid),
    .plic_icb_rsp_ready     (plic_icb_rsp_ready),
    .plic_icb_rsp_err       (plic_icb_rsp_err  ),
    .plic_icb_rsp_rdata     (plic_icb_rsp_rdata),

  // The PLIC Interface (ICB): End


  //////////////////////////////////////////////////////////////
  // The Fast IO Interface (ICB): Begin
  //
  //    * Bus cmd channel
  .fio_icb_cmd_valid     (fio_icb_cmd_valid),
  .fio_icb_cmd_ready     (fio_icb_cmd_ready),
  .fio_icb_cmd_addr      (fio_icb_cmd_addr ),
  .fio_icb_cmd_read      (fio_icb_cmd_read ),
  .fio_icb_cmd_wdata     (fio_icb_cmd_wdata),
  .fio_icb_cmd_wmask     (fio_icb_cmd_wmask),
  //    * Bus RSP channel
  .fio_icb_rsp_valid     (fio_icb_rsp_valid),
  .fio_icb_rsp_ready     (fio_icb_rsp_ready),
  .fio_icb_rsp_err       (fio_icb_rsp_err  ),
  .fio_icb_rsp_rdata     (fio_icb_rsp_rdata),
  // The Fast IO Interface (ICB): End

  //////////////////////////////////////////////////////////////
  // The System Memory Interface (ICB): Begin
  //
  //    * Bus cmd channel
  .mem_icb_cmd_valid  (mem_icb_cmd_valid),
  .mem_icb_cmd_ready  (mem_icb_cmd_ready),
  .mem_icb_cmd_addr   (mem_icb_cmd_addr ),
  .mem_icb_cmd_read   (mem_icb_cmd_read ),
  .mem_icb_cmd_wdata  (mem_icb_cmd_wdata),
  .mem_icb_cmd_wmask  (mem_icb_cmd_wmask),
  //    * Bus RSP channel
  .mem_icb_rsp_valid  (mem_icb_rsp_valid),
  .mem_icb_rsp_ready  (mem_icb_rsp_ready),
  .mem_icb_rsp_err    (mem_icb_rsp_err  ),
  .mem_icb_rsp_rdata  (mem_icb_rsp_rdata),
  // The System Memory Interface (ICB): End


  // The test mode signal
  .test_mode(test_mode), // in

  // The Clock
  .clk(clock), //in

  // The low-level active reset signal, treated as async
  .rst_n(reset) //in
  );
  
  // Implement RVFI
  /* rvfi_valid
    When the core retires an instruction, it asserts the rvfi_valid signal and 
    uses the signals described below to output the details of the retired instruction
  */
  // Two Stage Pipeline, IF || EXU.Decode EXU.Execute WB || Memory
  // if branch prediciton not wrong (IF) & PC_valid (EX), assume rvfi_valid
  // assign rvfi_valid = u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu_commit.alu_cmt_i_valid;
                      // &(!u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_ifu.ifu_halt_ack);
  logic [3:0] u_rvfi_mode;
  assign u_rvfi_mode[0] = u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.u_mode;
  assign u_rvfi_mode[1] = u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.s_mode;
  assign u_rvfi_mode[2] = u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.h_mode;
  assign u_rvfi_mode[3] = u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.m_mode;

  always_ff @(posedge clock ) begin : RVFI_Verify
    rvfi_valid <= reset && u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu_commit.alu_cmt_i_valid; // reset is active low
    rvfi_order <= reset ? rvfi_order + rvfi_valid : 0; // reset is active low

    rvfi_insn <= u_e203_cpu_top.u_e203_cpu.u_e203_core.ifu_o_ir;
    rvfi_trap <= u_e203_cpu_top.u_e203_cpu.u_e203_core.commit_trap;
    rvfi_halt <= u_e203_cpu_top.u_e203_cpu.u_e203_core.wfi_halt_ifu_req;
    rvfi_intr <= u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.u_e203_exu_commit.u_e203_exu_excp.irq_req_raw;
    rvfi_mode <= u_rvfi_mode;
    rvfi_ixl <= 1'b1; // XLEN=32
    rvfi_rs1_addr <= u_e203_cpu_top.u_e203_cpu.u_e203_core.ifu_o_rs1idx;
    rvfi_rs2_addr <= u_e203_cpu_top.u_e203_cpu.u_e203_core.ifu_o_rs2idx;
    rvfi_rs1_rdata <= u_e203_cpu_top.u_e203_cpu.u_e203_core.rf2ifu_rs1;
    rvfi_rs2_rdata <= u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.rf_rs2; // Not accessible at core level?
    rvfi_rd_addr <= u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.rf_wbck_rdidx;
    rvfi_rd_wdata <= u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.rf_wbck_wdat;
    
    rvfi_pc_rdata <= inspect_pc;
    rvfi_pc_wdata <= u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.alu_cmt_pc_vld ? 
                     u_e203_cpu_top.u_e203_cpu.u_e203_core.u_e203_exu.alu_cmt_pc : rvfi_pc_wdata;
    /* Memory does not need to be tested
    rvfi_mem_addr <= 
    rvfi_mem_rmask <= 
    rvfi_mem_wmask <= 
    rvfi_mem_rdata <= 
    rvfi_mem_wdata <= 
    */
  end

/* [To-Do]
  `define RVFI_CONN32 \
  .rvfi_valid     (rvfi_valid    ), \ check
  .rvfi_order     (rvfi_order    ), \ check
  .rvfi_insn      (rvfi_insn     ), \
  .rvfi_trap      (rvfi_trap     ), \ // ->u_e203_core.commit_trap
  .rvfi_halt      (rvfi_halt     ), \ // ->u_e203_core.wfi_halt_ifu_req
  .rvfi_intr      (rvfi_intr     ), \
  .rvfi_mode      (rvfi_mode     ), \ // ->u_e203_exu   wire u_mode;wire s_mode;wire h_mode;wire m_mode;
  .rvfi_ixl       (rvfi_ixl      ), \ <= 1'b1, XLEN=32
  .rvfi_rs1_addr  (rvfi_rs1_addr ), \ ->u_e203_core.ifu_o_rs1idx
  .rvfi_rs2_addr  (rvfi_rs2_addr ), \ ->u_e203_core.ifu_o_rs2idx 
  .rvfi_rs1_rdata (rvfi_rs1_rdata), \ ->u_e203_core.rf2ifu_rs1
  .rvfi_rs2_rdata (rvfi_rs2_rdata), \ ->u_e203_core.u_e203_exu.rf_rs2 (?)
  .rvfi_rd_addr   (rvfi_rd_addr  ), \ ->u_e203_core.u_e203_exu.rf_wbck_rdidx
  .rvfi_rd_wdata  (rvfi_rd_wdata ), \ ->u_e203_core.u_e203_exu.rf_wbck_wdat
  .rvfi_pc_rdata  (rvfi_pc_rdata ), \
  .rvfi_pc_wdata  (rvfi_pc_wdata ), \
  .rvfi_mem_addr  (rvfi_mem_addr ), \
  .rvfi_mem_rmask (rvfi_mem_rmask), \
  .rvfi_mem_wmask (rvfi_mem_wmask), \
  .rvfi_mem_rdata (rvfi_mem_rdata), \
  .rvfi_mem_wdata (rvfi_mem_wdata) \
  `rvformal_extamo_conn \
  `rvformal_rollback_conn \
  `rvformal_mem_fault_conn \
  `rvformal_csr_fflags_conn \
  `rvformal_csr_frm_conn \
  `rvformal_csr_fcsr_conn \
  `rvformal_csr_mvendorid_conn \
  `rvformal_csr_marchid_conn \
  `rvformal_csr_mimpid_conn \
  `rvformal_csr_mhartid_conn \
  `rvformal_csr_mconfigptr_conn \
  `rvformal_csr_mstatus_conn \
  `rvformal_csr_mstatush_conn \
  `rvformal_csr_misa_conn \
  `rvformal_csr_medeleg_conn \
  `rvformal_csr_mideleg_conn \
  `rvformal_csr_mie_conn \
  `rvformal_csr_mtvec_conn \
  `rvformal_csr_mcounteren_conn \
  `rvformal_csr_mscratch_conn \
  `rvformal_csr_mepc_conn \
  `rvformal_csr_mcause_conn \
  `rvformal_csr_mtval_conn \
  `rvformal_csr_mip_conn \
  `rvformal_csr_mtinst_conn \
  `rvformal_csr_mtval2_conn \
  `rvformal_csr_mcountinhibit_conn \
  `rvformal_csr_menvcfg_conn \
  `rvformal_csr_menvcfgh_conn \
  `rvformal_csr_pmpcfg0_conn \
  `rvformal_csr_pmpcfg1_conn \
  `rvformal_csr_pmpcfg2_conn \
  `rvformal_csr_pmpcfg3_conn \
  `rvformal_csr_pmpcfg4_conn \
  `rvformal_csr_pmpcfg5_conn \
  `rvformal_csr_pmpcfg6_conn \
  `rvformal_csr_pmpcfg7_conn \
  `rvformal_csr_pmpcfg8_conn \
  `rvformal_csr_pmpcfg9_conn \
  `rvformal_csr_pmpcfg10_conn \
  `rvformal_csr_pmpcfg11_conn \
  `rvformal_csr_pmpcfg12_conn \
  `rvformal_csr_pmpcfg13_conn \
  `rvformal_csr_pmpcfg14_conn \
  `rvformal_csr_pmpcfg15_conn \
  `rvformal_csr_pmpaddr0_conn \
  `rvformal_csr_pmpaddr1_conn \
  `rvformal_csr_pmpaddr2_conn \
  `rvformal_csr_pmpaddr3_conn \
  `rvformal_csr_pmpaddr4_conn \
  `rvformal_csr_pmpaddr5_conn \
  `rvformal_csr_pmpaddr6_conn \
  `rvformal_csr_pmpaddr7_conn \
  `rvformal_csr_pmpaddr8_conn \
  `rvformal_csr_pmpaddr9_conn \
  `rvformal_csr_pmpaddr10_conn \
  `rvformal_csr_pmpaddr11_conn \
  `rvformal_csr_pmpaddr12_conn \
  `rvformal_csr_pmpaddr13_conn \
  `rvformal_csr_pmpaddr14_conn \
  `rvformal_csr_pmpaddr15_conn \
  `rvformal_csr_pmpaddr16_conn \
  `rvformal_csr_pmpaddr17_conn \
  `rvformal_csr_pmpaddr18_conn \
  `rvformal_csr_pmpaddr19_conn \
  `rvformal_csr_pmpaddr20_conn \
  `rvformal_csr_pmpaddr21_conn \
  `rvformal_csr_pmpaddr22_conn \
  `rvformal_csr_pmpaddr23_conn \
  `rvformal_csr_pmpaddr24_conn \
  `rvformal_csr_pmpaddr25_conn \
  `rvformal_csr_pmpaddr26_conn \
  `rvformal_csr_pmpaddr27_conn \
  `rvformal_csr_pmpaddr28_conn \
  `rvformal_csr_pmpaddr29_conn \
  `rvformal_csr_pmpaddr30_conn \
  `rvformal_csr_pmpaddr31_conn \
  `rvformal_csr_pmpaddr32_conn \
  `rvformal_csr_pmpaddr33_conn \
  `rvformal_csr_pmpaddr34_conn \
  `rvformal_csr_pmpaddr35_conn \
  `rvformal_csr_pmpaddr36_conn \
  `rvformal_csr_pmpaddr37_conn \
  `rvformal_csr_pmpaddr38_conn \
  `rvformal_csr_pmpaddr39_conn \
  `rvformal_csr_pmpaddr40_conn \
  `rvformal_csr_pmpaddr41_conn \
  `rvformal_csr_pmpaddr42_conn \
  `rvformal_csr_pmpaddr43_conn \
  `rvformal_csr_pmpaddr44_conn \
  `rvformal_csr_pmpaddr45_conn \
  `rvformal_csr_pmpaddr46_conn \
  `rvformal_csr_pmpaddr47_conn \
  `rvformal_csr_pmpaddr48_conn \
  `rvformal_csr_pmpaddr49_conn \
  `rvformal_csr_pmpaddr50_conn \
  `rvformal_csr_pmpaddr51_conn \
  `rvformal_csr_pmpaddr52_conn \
  `rvformal_csr_pmpaddr53_conn \
  `rvformal_csr_pmpaddr54_conn \
  `rvformal_csr_pmpaddr55_conn \
  `rvformal_csr_pmpaddr56_conn \
  `rvformal_csr_pmpaddr57_conn \
  `rvformal_csr_pmpaddr58_conn \
  `rvformal_csr_pmpaddr59_conn \
  `rvformal_csr_pmpaddr60_conn \
  `rvformal_csr_pmpaddr61_conn \
  `rvformal_csr_pmpaddr62_conn \
  `rvformal_csr_pmpaddr63_conn \
  `rvformal_csr_mhpmevent3_conn \
  `rvformal_csr_mhpmevent4_conn \
  `rvformal_csr_mhpmevent5_conn \
  `rvformal_csr_mhpmevent6_conn \
  `rvformal_csr_mhpmevent7_conn \
  `rvformal_csr_mhpmevent8_conn \
  `rvformal_csr_mhpmevent9_conn \
  `rvformal_csr_mhpmevent10_conn \
  `rvformal_csr_mhpmevent11_conn \
  `rvformal_csr_mhpmevent12_conn \
  `rvformal_csr_mhpmevent13_conn \
  `rvformal_csr_mhpmevent14_conn \
  `rvformal_csr_mhpmevent15_conn \
  `rvformal_csr_mhpmevent16_conn \
  `rvformal_csr_mhpmevent17_conn \
  `rvformal_csr_mhpmevent18_conn \
  `rvformal_csr_mhpmevent19_conn \
  `rvformal_csr_mhpmevent20_conn \
  `rvformal_csr_mhpmevent21_conn \
  `rvformal_csr_mhpmevent22_conn \
  `rvformal_csr_mhpmevent23_conn \
  `rvformal_csr_mhpmevent24_conn \
  `rvformal_csr_mhpmevent25_conn \
  `rvformal_csr_mhpmevent26_conn \
  `rvformal_csr_mhpmevent27_conn \
  `rvformal_csr_mhpmevent28_conn \
  `rvformal_csr_mhpmevent29_conn \
  `rvformal_csr_mhpmevent30_conn \
  `rvformal_csr_mhpmevent31_conn \
  `rvformal_csr_mcycle_conn32 \
  `rvformal_csr_time_conn32 \
  `rvformal_csr_minstret_conn32 \
  `rvformal_csr_mhpmcounter3_conn32 \
  `rvformal_csr_mhpmcounter4_conn32 \
  `rvformal_csr_mhpmcounter5_conn32 \
  `rvformal_csr_mhpmcounter6_conn32 \
  `rvformal_csr_mhpmcounter7_conn32 \
  `rvformal_csr_mhpmcounter8_conn32 \
  `rvformal_csr_mhpmcounter9_conn32 \
  `rvformal_csr_mhpmcounter10_conn32 \
  `rvformal_csr_mhpmcounter11_conn32 \
  `rvformal_csr_mhpmcounter12_conn32 \
  `rvformal_csr_mhpmcounter13_conn32 \
  `rvformal_csr_mhpmcounter14_conn32 \
  `rvformal_csr_mhpmcounter15_conn32 \
  `rvformal_csr_mhpmcounter16_conn32 \
  `rvformal_csr_mhpmcounter17_conn32 \
  `rvformal_csr_mhpmcounter18_conn32 \
  `rvformal_csr_mhpmcounter19_conn32 \
  `rvformal_csr_mhpmcounter20_conn32 \
  `rvformal_csr_mhpmcounter21_conn32 \
  `rvformal_csr_mhpmcounter22_conn32 \
  `rvformal_csr_mhpmcounter23_conn32 \
  `rvformal_csr_mhpmcounter24_conn32 \
  `rvformal_csr_mhpmcounter25_conn32 \
  `rvformal_csr_mhpmcounter26_conn32 \
  `rvformal_csr_mhpmcounter27_conn32 \
  `rvformal_csr_mhpmcounter28_conn32 \
  `rvformal_csr_mhpmcounter29_conn32 \
  `rvformal_csr_mhpmcounter30_conn32 \
  `rvformal_csr_mhpmcounter31_conn32 \
  `rvformal_custom_csr_conn
*/
endmodule