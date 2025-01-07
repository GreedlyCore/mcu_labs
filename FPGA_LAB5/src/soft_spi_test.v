`timescale 1ms/100us 
module soft_spi_test;
  reg clk = 0;
  reg ss;
  reg [7:0] data;
  always #5 clk = clk + 1;
  
//  module Gowin_rPLL (clkout, clkoutd, clkin);
  soft_spi top0(.clk(clk), .data(data), .ss(ss));
  
  initial begin
    $dumpfile("wave.vcd");  
    $dumpvars(0, soft_spi_test);  
        # 10000 $finish();
  end

  initial begin
    $monitor("t=%-4d: clk = %d, data = %d, ss = %d", $time, clk, data, ss);
  end
endmodule
