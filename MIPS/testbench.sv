// Code your testbench here
// or browse Examples
module tb;
  wire[31:0] PC_in;
  reg clk, rst;
  wire[4:0] mux_WriteReg;
  wire[31:0] PC_add4_out, alu_out;
  wire[1:0] ALUOp;
  wire[27:0] jump_addr;
  wire[31:0] pc_instr,  ReadData1, ReadData2, sign_extended_out, mux_ALU_input, datamem_out, MemToReg_data, sl2_out, branch_addr, branch_mux_out;
  wire Jump, Branch,  MemToReg, MemWrite, ALUSrc, RegWrite, zero_out, branch_out, RegDst;
  wire[3:0] ALUC_out;
  
  
  
  MIPS mips1( rst,  pc_instr, clk,RegDst, Jump, Branch,  MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite, mux_WriteReg, PC_add4_out, jump_addr, ReadData1, ReadData2, sign_extended_out, ALUC_out, mux_ALU_input, zero_out, alu_out, datamem_out, MemToReg_data, branch_out, sl2_out, branch_addr, branch_mux_out);


  
  initial begin clk=1'b1; rst=1'b1; end
  initial forever #5 clk=~clk;
  integer i;
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    for(i=0;i<32;i=i+1) begin
      $dumpvars(0, mips1.registri.registri[i]);
      $dumpvars(0, mips1.dm.dmem[i]);
    end

    #10 rst=1'b0;
    #300 $finish;
  end
endmodule