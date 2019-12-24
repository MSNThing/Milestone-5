`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// Pipelined MIPS processor

module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        ID_signext, ID_shiftl16, WB_memtoreg, EX_regwrite, MEM_regwrite, EX_memread;
  wire        ID_pcsrc, ID_equal;
  wire        EX_alusrc, EX_regdst, WB_regwrite, ID_jump, EX_jal, WB_jal, ID_jr;
  wire [2:0]  EX_alucontrol;
  
  // ##### Hyuna Kim : Start #####
  wire		  EX_flush;
  wire [5:0]  ID_op, ID_funct;  
  // ##### Hyuna Kim : End #####
  
  // Instantiate Controller
 controller c(
    .clk           (clk),
	  .reset         (reset),
    .ID_op         (ID_op), 
		.ID_funct      (ID_funct),
		.ID_signext    (ID_signext),
		.ID_shiftl16   (ID_shiftl16),
		.WB_memtoreg   (WB_memtoreg),
		.EX_memread    (EX_memread),
		.MEM_memwrite  (memwrite),
		.ID_pcsrc      (ID_pcsrc),
		.ID_equal		    (ID_equal),
		.EX_alusrc     (EX_alusrc),
		.EX_regdst     (EX_regdst),
		.EX_regwrite   (EX_regwrite),
		.MEM_regwrite  (MEM_regwrite),
		.WB_regwrite   (WB_regwrite),
		.ID_jump       (ID_jump),
		.EX_flush      (EX_flush),
		.EX_jal 			    (EX_jal),
		.WB_jal 	  	   (WB_jal),
		.ID_jr		       (ID_jr),
		.EX_alucontrol (EX_alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk           (clk),
    .reset         (reset),
    .ID_signext    (ID_signext),
    .ID_shiftl16   (ID_shiftl16),
    .WB_memtoreg   (WB_memtoreg),
    .ID_pcsrc      (ID_pcsrc),
	  .ID_equal		 (ID_equal),
    .EX_alusrc     (EX_alusrc),
    .EX_regdst     (EX_regdst),
	  .EX_memread    (EX_memread),
	  .EX_regwrite   (EX_regwrite),
    .WB_regwrite   (WB_regwrite),
	  .MEM_regwrite  (MEM_regwrite),
    .ID_jump       (ID_jump),
	  .EX_jal			 (EX_jal),
	  .EX_flush 		 (EX_flush),
	  .WB_jal			 (WB_jal),
	  .ID_jr			 (ID_jr),
    .EX_alucontrol (EX_alucontrol),
	  .ID_op         (ID_op),
	  .ID_funct      (ID_funct),
    .IF_pc         (pc),
    .IF_instr      (instr),
    .MEM_aluout    (memaddr), 
    .MEM_writedata (memwritedata),
    .MEM_readdata  (memreaddata));

endmodule


module controller(input        clk, reset,
					   input  [5:0] ID_op, ID_funct,
						input        EX_flush,
						input	       ID_equal,
						output       ID_signext,
                  output       ID_shiftl16,
                  output       WB_memtoreg, MEM_memwrite, EX_memread,
                  output       ID_pcsrc, 
						output       EX_alusrc,
                  output       EX_regdst, EX_regwrite, MEM_regwrite, WB_regwrite,
                  output       ID_jump, EX_jal, WB_jal, ID_jr,
                  output [2:0] EX_alucontrol);

  wire [1:0] ID_aluop, ID_branch;
  wire		 ID_memtoreg, ID_memread, ID_memwrite, ID_alusrc, ID_regdst, ID_regwrite, ID_jal;
  wire       EX_jr, MEM_jal, MEM_jr;
  wire       EX_memtoreg, MEM_memtoreg;
  wire       EX_memwrite;
  wire [2:0] ID_alucontrol;

  maindec md(
    .op       (ID_op),
	 .funct	  (ID_funct),
    .signext  (ID_signext),
    .shiftl16 (ID_shiftl16),
    .memtoreg (ID_memtoreg),
	 .memread  (ID_memread),
    .memwrite (ID_memwrite),
    .branch   (ID_branch),
    .alusrc   (ID_alusrc),
    .regdst   (ID_regdst),
    .regwrite (ID_regwrite),
    .jump     (ID_jump),
	 .jal		  (ID_jal),
	 .jr		  (ID_jr),
    .aluop    (ID_aluop));

  aludec ad( 
    .funct      (ID_funct),
    .aluop      (ID_aluop), 
    .alucontrol (ID_alucontrol));
	
  // ##### Hyuna Kim : Start #####
  // FF ID>EX
  floprc #(11) EX_reg(clk, reset, EX_flush,
                     {ID_memtoreg, ID_memwrite, ID_memread, ID_alusrc, ID_regdst, ID_regwrite, ID_jr, ID_jal, ID_alucontrol},
                     {EX_memtoreg, EX_memwrite, EX_memread, EX_alusrc, EX_regdst, EX_regwrite, EX_jr, EX_jal, EX_alucontrol});
		  
  // FF EX>MEM
  flopr  #(5) MEM_reg(clk, reset,
                     {EX_memtoreg, EX_memwrite, EX_regwrite, EX_jr, EX_jal},
                     {MEM_memtoreg, MEM_memwrite, MEM_regwrite, MEM_jr, MEM_jal});		  
  // FF MEM>WB
  flopr  #(3) WB_reg(clk, reset,
                     {MEM_memtoreg, MEM_regwrite, MEM_jal},
                     {WB_memtoreg, WB_regwrite, WB_jal});            

  assign ID_pcsrc = ID_branch[1] ? (ID_branch[0] ? (ID_branch[0] & ID_equal) : (~ID_branch[0] & ~ID_equal)) : (0);

  // ##### Hyuna Kim : End #####

endmodule

// Datapath
module datapath(input         clk, reset,
                input         ID_signext,
                input         ID_shiftl16,
                input         WB_memtoreg,
					      input         ID_pcsrc,
                input         EX_alusrc, EX_regdst, EX_memread,
                input         EX_regwrite, MEM_regwrite, WB_regwrite, ID_jump, EX_jal, WB_jal, ID_jr,
                input  [2:0]  EX_alucontrol,
					      output [5:0]  ID_op, ID_funct,
					      output			EX_flush,
                output [31:0] IF_pc,
				      	 output		   ID_equal,
                input  [31:0] IF_instr,
                output [31:0] MEM_aluout, MEM_writedata,
                input  [31:0] MEM_readdata);

  wire [1:0]  ID_forwarda, ID_forwardb;
  wire [1:0]  EX_forwarda, EX_forwardb;
  wire		      IF_stall, ID_stall;
  wire [31:0] ID_instr, EX_instr, MEM_instr;
  wire [31:0] ID_signimm, ID_shiftedimm, ID_signimmsh, EX_shiftedimm;
  wire [4:0]  EX_writesubreg;
  wire [4:0]  EX_writereg, MEM_writereg, WB_writereg;
  wire [31:0] WB_result;
  wire 	      EX_zero;
  wire [31:0] EX_aluout, WB_aluout;
  wire [31:0] EX_alua, EX_alub;
  wire [31:0] WB_readdata;
  wire [31:0] WB_subresult;
  wire [31:0] IF_pcnext, IF_pcplus4, ID_pcplus4, EX_pcplus4, MEM_pcplus4, WB_pcplus4;
  wire [31:0] ID_pcbranch, MEM_pcbranch;
  wire [31:0] ID_pcnextbr, ID_pcnextj;
  wire [31:0] ID_srca, ID_subsrca, EX_srca, MEM_srca; 
  wire [31:0] EX_srcb;
  wire [31:0] ID_subwritedata, ID_writedata, EX_writedata;
  wire [31:0] EX_subwritedata;
  wire	   	  ID_flush;
 
  // ##### Hyuna Kim : Start #####
  
  Forwarding f(
   .ID_rs	(ID_instr[25:21]),
   .ID_rt   (ID_instr[20:16]),
   .EX_rs	(EX_instr[25:21]),
	.EX_rt	(EX_instr[20:16]),
	.EX_rd   (EX_writereg),
	.MEM_rd	(MEM_writereg),
	.WB_rd	(WB_writereg),
	.EX_regwrite  (EX_regwrite),
	.MEM_regwrite (MEM_regwrite),
	.WB_regwrite  (WB_regwrite),
	.ID_forwarda  (ID_forwarda),
	.ID_forwardb  (ID_forwardb),
   .EX_forwarda  (EX_forwarda),
	.EX_forwardb  (EX_forwardb));
	
HazardDetection h(
	.ID_rs (ID_instr[25:21]),
	.ID_rt (ID_instr[20:16]),
	.EX_rt (EX_instr[20:16]),
	.EX_memread (EX_memread),
	.IF_stall (IF_stall),
	.ID_stall (ID_stall),
	.EX_flush (EX_flush));
  
  // ##### Hyuna Kim : End #####
  
  // Next PC Logic	 
    mux2 #(32) pcbrmux(
    .d0  (IF_pcplus4),
    .d1  (ID_pcbranch),
    .s   (ID_pcsrc),
    .y   (ID_pcnextbr));
	 
  mux2 #(32) pcjmux(
    .d0   (ID_pcnextbr),
    .d1   ({ID_pcplus4[31:28], ID_instr[25:0], 2'b00}),
    .s    (ID_jump),
    .y    (ID_pcnextj));

  mux2 #(32) pcmux(  // for jr instruction (pc <- ra, srca is a rs register result)
    .d0   (ID_pcnextj),
    .d1   (ID_srca),
    .s    (ID_jr),
    .y    (IF_pcnext));
  
  // Register File
  regfile rf(
    .clk     (clk),
    .we      (WB_regwrite),
    .ra1     (ID_instr[25:21]),
    .ra2     (ID_instr[20:16]),
    .wa      (WB_writereg),
    .wd      (WB_result),
    .rd1     (ID_subsrca),
    .rd2     (ID_subwritedata));
  
  // Fetch stage (flopenr for stall)
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
	 .en    (~IF_stall),
    .d     (IF_pcnext),
    .q     (IF_pc));
	 
  adder pcadd1(
    .a (IF_pc),
    .b (32'b100),
    .y (IF_pcplus4));


  // ##### Hyuna Kim : Start #####
  
  // Decoding stage - FF IF>ID
  flopenrc #(32) ID_r1 (clk, reset, ~ID_stall, ID_flush, IF_instr, ID_instr);
  flopenr  #(32) ID_r2 (clk, reset, ~ID_stall, IF_pcplus4, ID_pcplus4);
  
  // ##### Hyuna Kim : End #####

   sign_zero_ext sze(
    .a       (ID_instr[15:0]),
    .signext (ID_signext),
    .y       (ID_signimm[31:0]));
  
  shift_left_16 sl16(
    .a         (ID_signimm[31:0]),
    .shiftl16  (ID_shiftl16),
    .y         (ID_shiftedimm[31:0]));
	 
  sl2 immsh(
    .a (ID_signimm),
    .y (ID_signimmsh));
	 
  adder pcadd2(
    .a (ID_pcplus4),
    .b (ID_signimmsh),
    .y (ID_pcbranch));

  // ##### Hyuna Kim : Start #####
  
  mux3 #(32) forwardadmux( 
    .d0  (ID_subsrca),
    .d1  (WB_result),
	 .d2  (EX_aluout),
    .s   (ID_forwarda),
    .y   (ID_srca));
  
   mux3 #(32) forwardbdmux( 
    .d0  (ID_subwritedata),
    .d1  (WB_result),
	 .d2  (EX_aluout),
    .s   (ID_forwardb),
    .y   (ID_writedata));
    
    eq eqcomp(
	 .a   (ID_srca),
	 .b	(ID_writedata),
	 .eq  (ID_equal));
  
  assign ID_op = ID_instr[31:26];
  assign ID_funct = ID_instr[5:0];
  assign ID_flush = ID_pcsrc | ID_jump;
  
  // Execution stage - FF ID>EX
 floprc #(32) EX_r1 (clk, reset, EX_flush, ID_instr, EX_instr);
  floprc #(32) EX_r2 (clk, reset, EX_flush, ID_pcplus4, EX_pcplus4);
  floprc #(32) EX_r3 (clk, reset, EX_flush, ID_writedata, EX_subwritedata);
  floprc #(32) EX_r4 (clk, reset, EX_flush, ID_shiftedimm, EX_shiftedimm);
  floprc #(32) EX_r5 (clk, reset, EX_flush, ID_srca, EX_srca);
  
  // ##### Hyuna Kim : End #####
	 
  mux2 #(5) wrsubmux(  // select rt or rd for destination
    .d0  (EX_instr[20:16]),
    .d1  (EX_instr[15:11]),
    .s   (EX_regdst),
    .y   (EX_writesubreg));
	 
  mux2 #(5) wrmux(  // select writesubreg or ra($r31) reg (for jal instruction: ra <- pc + 4)
    .d0  (EX_writesubreg),
    .d1  (5'b11111),
    .s   (EX_jal),
    .y   (EX_writereg));
  
  // ##### Hyuna Kim : Start #####
 
  mux2 #(32) forwardlwemux (EX_subwritedata, WB_readdata, EX_forwardb[0], EX_writedata);
  mux3 #(32) forwardaemux(EX_srca, WB_result, MEM_aluout, EX_forwarda, EX_alua);
  mux3 #(32) forwardbemux(EX_writedata, WB_result, MEM_aluout, EX_forwardb, EX_srcb);
 
  // ##### Hyuna Kim : end #####
  
   mux2 #(32) srcbmux(
    .d0 (EX_srcb),
    .d1 (EX_shiftedimm[31:0]),
    .s  (EX_alusrc),
    .y  (EX_alub));
  
  alu alu(
    .a       (EX_alua),
    .b       (EX_alub),
    .alucont (EX_alucontrol),
    .result  (EX_aluout),
    .zero    (EX_zero));
  
  // ##### Hyuna Kim : Start #####
  
  // Memory Access stage - FF EX>MEM
  flopr #(32) MEM_r1 (clk, reset, EX_instr, MEM_instr);
  flopr #(32) MEM_r2 (clk, reset, EX_pcplus4, MEM_pcplus4);
  flopr #(5)  MEM_r3 (clk, reset, EX_writereg, MEM_writereg);
  flopr #(32) MEM_r4 (clk, reset, EX_aluout, MEM_aluout);
  flopr #(32) MEM_r5 (clk, reset, EX_writedata, MEM_writedata);
  flopr #(32) MEM_r6 (clk, reset, EX_srca, MEM_srca);
  
  // Write Back stage - FF MEM>WB 
  flopr #(5)  WB_r1 (clk, reset, MEM_writereg, WB_writereg);
  flopr #(32) WB_r2 (clk, reset, MEM_readdata, WB_readdata);
  flopr #(32) WB_r3 (clk, reset, MEM_aluout, WB_aluout);
  flopr #(32) WB_r4 (clk, reset, MEM_pcplus4, WB_pcplus4);
  
  // ##### Hyuna Kim : End #####
  
  mux2 #(32) resmux( 
    .d0 (WB_aluout),
    .d1 (WB_readdata),
    .s  (WB_memtoreg),
    .y  (WB_subresult));

  mux2 #(32) jalresmux( 
    .d0 (WB_subresult),
    .d1 (WB_pcplus4),
    .s  (WB_jal),
    .y  (WB_result));	  


endmodule

// ##### Hyuna Kim : Start #####

module Forwarding(input  [4:0]     ID_rs, ID_rt, EX_rs, EX_rt, EX_rd, MEM_rd, WB_rd,
				      input 	 	     EX_regwrite, MEM_regwrite, WB_regwrite,
						output reg [1:0] ID_forwarda, ID_forwardb,
				      output reg [1:0] EX_forwarda, EX_forwardb);
				  
  always @(*)
  begin
		ID_forwarda = 2'b00;
		ID_forwardb = 2'b00;
		EX_forwarda = 2'b00;
		EX_forwardb = 2'b00;
		if (ID_rs != 0)
			if (ID_rs == WB_rd & WB_regwrite) ID_forwarda = 2'b01;
			else if (ID_rs == EX_rd & EX_regwrite) ID_forwarda = 2'b10;
		if (ID_rt != 0)
			if (ID_rt == WB_rd & WB_regwrite) ID_forwardb = 2'b01;
			else if (ID_rt == EX_rd & EX_regwrite) ID_forwardb = 2'b10;
		if (EX_rs != 0)
			if (EX_rs == MEM_rd & MEM_regwrite) EX_forwarda = 2'b10;
			else if (EX_rs == WB_rd & WB_regwrite) EX_forwarda = 2'b01;
		if (EX_rt != 0)
			if (EX_rt == MEM_rd & MEM_regwrite) EX_forwardb = 2'b10;
			else if (EX_rt == WB_rd & WB_regwrite) EX_forwardb = 2'b01;
  end

endmodule

module HazardDetection(input [4:0] ID_rs, ID_rt, EX_rt,
							  input		  EX_memread,
							  output      IF_stall, ID_stall, EX_flush);

  assign IF_stall = EX_memread & (EX_rt == ID_rs | EX_rt == ID_rt);
  assign ID_stall = IF_stall;
  assign EX_flush = IF_stall;

endmodule


// ##### Hyuna Kim : End #####