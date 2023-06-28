// Code your design here
module PIPO(Clk,Pi,Po);
  input Clk;
  input [3:0]Pi;
  output reg [3:0]Po;
  always @(posedge Clk)
  begin
     Po <= Pi;
  end
endmodule

module SIPO(clk,Si,So);
  input Si;
  input clk;
  output reg [3:0]So;
  always@(posedge clk)
  begin
  
    
    So[3]<= Si;
    So[2]<=So[3];
    So[1]<=So[2];
    So[0]<=So[1];
	end
endmodule

module SISO(clk, Si, So);
  input clk, Si;
  output So;
  reg [3:0]q =0;
  
  always@(posedge clk)
  begin
  
    
    q[3]<= Si;
    q[2]<=q[3];
    q[1]<=q[2];
    q[0]<=q[1];
	end
  assign So = q[0];
  
endmodule
  

