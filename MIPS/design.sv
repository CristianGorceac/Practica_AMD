// Code your design here
module PC(d_i, d_o, clk, rst);
  input [31:0]d_i;
  input  clk, rst;
  output reg [31:0]d_o;
  
  always @(posedge clk or posedge rst) begin
    if(rst)
      d_o=32'd0;
    else
    
       d_o <= d_i;
  end
    
endmodule

// Code your design here
module InstructionMemory(PC_out, instruction);
  input [31:0]PC_out;
  output reg [31:0]instruction;
  
  reg[31:0] mem[0:1023];
  initial begin
    $readmemb("instr1.txt", mem);
    
  end
  always @* begin
    
    instruction = mem[PC_out >> 2];
    
  end
endmodule
  
// Code your design here
module Control(opcode, RegDst, Jump, Branch, MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite);
  input [5:0]opcode;
  output reg  RegDst, Jump, Branch, MemToReg, MemWrite, ALUSrc, RegWrite;
  output reg[1:0] ALUOp;
  
  
  always @* begin
    casex(opcode)
      6'b000000: begin // R-type
        RegDst =1'b1; ALUOp=2'b10; RegWrite=1'b1;
        Jump= 1'b0; ALUSrc= 1'b0; MemWrite= 1'b0; MemToReg= 1'b0;  Branch = 1'b0;
      end
      
      6'b000010 : begin //j
        RegDst=1'bx; Branch=1'bx;  MemToReg=1'bx; ALUOp=2'bxx; MemWrite=1'b0; ALUSrc=1'bx; RegWrite=1'b0;
        Jump = 1'b1;
      end
      6'b000100  : begin // branch
        Branch = 1'b1; ALUOp = 2'b01;
        RegDst=1'bx; Jump=1'b0;  MemToReg=1'bx; MemWrite=1'b0; ALUSrc=1'b0; RegWrite = 1'b0;
      end
      6'b101011 : begin //sw
        ALUOp = 2'b00; MemWrite = 1'b1; ALUSrc=1'b1;
        RegDst=1'bx; Jump=1'b0; Branch=1'b0;  MemToReg=1'bx; RegWrite=1'b0;
      end
      6'b100011: begin //lw
         MemToReg = 1'b1; ALUOp = 2'b00; ALUSrc = 1'b1; RegWrite = 1'b1;
        RegDst=1'b0; Jump=1'b0; Branch=1'b0; MemWrite=1'b0;
        
      end
      6'b001000 : begin //addi
        MemToReg = 1'b0; ALUOp = 2'b00; ALUSrc = 1'b1; RegWrite = 1'b1;
        RegDst=1'b0; Jump=1'b0; Branch=1'b0; MemWrite=1'b0;
      
        
      end
      
    endcase
    
      
  end
endmodule
  
// Code your design here
module mux21_5(sel, rt, rd, dout);
  input sel;
  input[4:0] rt, rd;
  output reg[4:0] dout;
  
  always @* begin
    dout= (sel) ? rd : rt;
  end
  
endmodule

// Code your design here
module adder(src1, src2, dout);
  input[31:0] src1, src2;
  output reg[31:0] dout;
  
  
  always @* 
    dout <= src1+src2;
endmodule

// Code your design here
module shift_left26_28(din, dout);
  input[25:0] din;
  output reg[27:0] dout;
  
  
  always @* begin
    dout <= {din, 2'b00};
  end
  
endmodule
// Code your design here
module mux21_32(sel, din0, din1, dout);
  input sel;
  input[31:0] din0, din1;
  output reg[31:0] dout;
  
  always @* begin
    dout= (sel) ? din1 : din0;
  end
  
endmodule

// Code your design here
module Registers(rs, rt, writer, readd1, readd2, writed, RegWrite, clk);
	
  input RegWrite, clk;
  input[4:0] rs, rt, writer;
  input[31:0] writed;
  output reg[31:0] readd1, readd2;
  

  reg [31:0] registri [0:31];
	integer i;
  
  initial  begin
    for(i=0;i<32;i=i+1) begin
      registri[i]=32'd0;
    end
  end
  
  always @(rs, rt, posedge clk) begin
    readd1 = registri[rs];  
    readd2 = registri[rt];    
  end

  always @(negedge clk) begin
    if (RegWrite)
      registri[writer] <= writed;
  end

endmodule

  

// Code your design here
module sign_extend(instr, sign_imm);
  input [15:0]instr;
  output reg [31:0]sign_imm;
  always @(*) begin
    sign_imm[15:0] = instr[15:0];
    sign_imm[31:16] = {16{instr[15]}} ;
      
    
    
      
  end
    
endmodule

// Code your design here
module ALUControl(func, ALUOp, ALUC_out);
  input[5:0] func;
  input[1:0] ALUOp;
  output reg[3:0] ALUC_out;
  
  
  
  always @(*) begin
    casex(ALUOp) 
      2'b00: ALUC_out<=4'b0010; //add(sw, lw)
      2'b01: ALUC_out<=4'b0110; //sub(beq)
      default: casex(func) 
        6'b100100: ALUC_out<=4'b0000;//and
        6'b100101: ALUC_out<=4'b0001;//or
        6'b100000: ALUC_out<=4'b0010;//add
        6'b100010: ALUC_out<=4'b0110;//sub
        6'b101010: ALUC_out<=4'b0111;//slt
        6'b100111: ALUC_out<=4'b1100;//nor
        6'b100110: ALUC_out<=4'b0011;//xor
        default: ALUC_out<=4'bxxxx;
      
      endcase
    endcase
  end
  
endmodule
   

// Code your design here
// Code your design here
module ALU(src1, src2, aluctrl,  zero, rez);
  input[31:0] src1, src2;
  input[3:0] aluctrl;
  output reg[31:0] rez;
  output reg zero;
  
  always @(src1, src2, aluctrl) begin
    case(aluctrl)
      4'bxxxx: rez<=32'hx;
      4'b0000: rez <= src1 & src2;//and
  	  4'b0001: rez <= src1 | src2;//or
      4'b0010: rez <= src1 + src2;//add
      4'b0110: rez <= src1 - src2;//sub
      4'b0111: begin //slt
        if(src1[31] != src2[31])
          rez <= (src1[31] > src2[31]) ? 1 : 0;
        else
          rez<= (src1 < src2) ? 1 : 0; 
      end
      4'b1100: rez<=~(src1 | src2);//nor
      4'b0011: rez <= src1 ^ src2;//xor
      
      
      
    endcase
    
  end
  
  always @(rez) begin
    if(rez == 0)
      zero <= 1'b1;
    else
      zero <= 1'b0;
  end
endmodule

// Code your design here
module DataMemory(addr, wd,  clk, rd, memwrite);
  input[31:0] addr, wd;
  input clk, memwrite;
  output reg[31:0] rd;
  
  reg[31:0] dmem[0:127];
  always @(negedge clk, addr)
    rd <= dmem[addr[31:2]];
  
  always @(negedge clk) begin
    
    if(memwrite)
      dmem[addr[31:2]] <= wd;
     
       
       
  end
  
endmodule
  

// Code your design here
module shiftleft2(din, dout);
  input[31:0] din;
  output reg[31:0] dout;
  
  always @* 
    dout <= din << 2;
  
endmodule
  

module MIPS( rst,pc_instr,  clk,RegDst,  Jump, Branch,  MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite, mux_out, add4out, shift_out, readd1, readd2, sign_extend_out, ALUC_out, mux2132_out, zero_out, alu_out, datamem_out, mux_memreg_out, branch_out, sl2_out, add4addr_out, branch_mux_out);

    

  input clk, rst;
  inout RegWrite , ALUSrc, MemWrite, MemToReg , Branch, zero_out, branch_out, Jump, RegDst;
  inout[1:0] ALUOp;
  inout[4:0] mux_out;
  inout[31:0] add4addr_out;
  inout[27:0] shift_out;
  inout[31:0] pc_instr, branch_mux_out;
  wire[31:0] instr;
  wire RegDst;
  inout[31:0] readd1, readd2 ,mux2132_out , sign_extend_out, alu_out, sl2_out, add4out;  
  inout[3:0] ALUC_out;
  inout[31:0] datamem_out, mux_memreg_out;
  inout [31:0] PC_in;
  
 
  
  
  PC pct(.d_i(PC_in), .d_o(pc_instr), .clk(clk), .rst(rst));
  InstructionMemory instrmem(.PC_out(pc_instr), .instruction(instr));
  Control control(.opcode(instr[31:26]), .RegDst(RegDst), .Jump(Jump), .Branch(Branch),  .MemToReg(MemToReg),.ALUOp(ALUOp) ,.MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
  mux21_5 mux215(.sel(RegDst), .rt(instr[20:16]), .rd(instr[15:11]), .dout(mux_out));
  adder add4(.src1(pc_instr), .src2(32'd4), .dout(add4out));
  shift_left26_28 shift2628(.din(instr[25:0]), .dout(shift_out));
  Registers registri(.rs(instr[25:21]), .rt(instr[20:16]), .writer(mux_out), .readd1(readd1), .readd2(readd2), .writed(mux_memreg_out), .RegWrite(RegWrite), .clk(clk));
  sign_extend se(.instr(instr[15:0]), .sign_imm(sign_extend_out));
  ALUControl aluc(.func(instr[5:0]), .ALUOp(ALUOp[1:0]), .ALUC_out(ALUC_out));
  mux21_32 mux2132(.sel(ALUSrc), .din0(readd2), .din1(sign_extend_out), .dout(mux2132_out));
  ALU alu(.src1(readd1), .src2(mux2132_out), .aluctrl(ALUC_out),  .zero(zero_out), .rez(alu_out));
  DataMemory dm(.addr(alu_out), .wd(readd2),  .clk(clk), .rd(datamem_out), .memwrite(MemWrite));
  mux21_32 mux_memtoreg(.sel(MemToReg), .din0(alu_out), .din1(datamem_out), .dout(mux_memreg_out));
  and(branch_out, Branch, zero_out);
  shiftleft2 sl2(.din(sign_extend_out), .dout(sl2_out));
  adder add4addr(.src1(add4out), .src2(sl2_out), .dout(add4addr_out));
  mux21_32 branch_mux(.sel(branch_out), .din0(add4out), .din1(add4addr_out), .dout(branch_mux_out));
  mux21_32 jump_mux(.sel(Jump), .din0(branch_mux_out), .din1({add4out[31:28], shift_out}), .dout(PC_in));
  
endmodule
      
  