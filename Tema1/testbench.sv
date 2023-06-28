// Code your testbench here
// or browse Examples
/*
module testbench();
 
  PIPO test(clkt, Pit, Pot);
  
  reg clkt;
  reg[3:0] Pit;
  
  
  wire[3:0]  Pot;
  
  initial Pit= 4'b0101;
  initial clkt=1'b0;
 
  
  initial forever #10 clkt = ~clkt;
  
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    #15 Pit=4'b1111;
    #300
    $finish(1);
  end
  
  
endmodule
*/

module testbench();
 
  SIPO test(clkt, Sit, Pot);
  
  reg clkt;
  reg Sit;
  
  
  wire[3:0]  Pot;
  
  initial Sit= 1'b0;
  initial clkt=1'b0;
 
  
  initial forever #10 clkt = ~clkt;
  
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    
    #10 Sit= 1'b0;
    #10 Sit= 1'b1;
    #10 Sit= 1'b1;
    #300
    $finish(1);
  end
  
  
endmodule