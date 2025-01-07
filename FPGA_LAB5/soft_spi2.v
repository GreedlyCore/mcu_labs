// module soft_spi (
//     input clk,         // System clock
//     output reg data,   // Data output
//     output reg ss,     // Slave select output
//     output clk_out, // SPI clock output
//     output IO_voltage
// );
    
//     led blinking_led(.clk(clk), .IO_voltage(IO_voltage));
    
//     parameter count_value       = 1_000_000; // some time ???
//     reg [7:0] d = 0;       // Data to transmit
//     reg [2:0] i = 0;       // Bit index for transmission
//     reg [20:0] sum = 0;    // Counter for timing
//     reg [2:0] number = 0;  // Current digit index
//     reg spi_active = 0;    // SPI transmission active flag

//     assign clk_out = clk;  // Direct system clock as SPI clock (simplified)

//     always @(posedge clk) begin
//         sum <= sum + 1'b1;
        
//         if (sum == 3_000_000) begin
//             sum <= 0;
//             if (!spi_active) begin
//                 // Begin new SPI transmission
//                 spi_active <= 1;
//                 ss <= 0;   // Activate slave select
//                 i <= 0;    // Reset bit counter
//                 case (number)
//                     0: d <= 8'h3F;
//                     1: d <= 8'h06;
//                     2: d <= 8'h5B;
//                     3: d <= 8'h4F;
//                     4: d <= 8'h66;
//                     5: d <= 8'h6D;
//                     6: d <= 8'h7D;
//                     7: d <= 8'h07;
//                     8: d <= 8'h7F;
//                     9: d <= 8'h6F;
//                     default: d <= 8'h00;
//                 endcase
//                 if (number == 10) number <= 0;
//                 else number <= number + 1'b1;
//             end else begin
//                 // Continue SPI transmission
//                 data <= d[i];  // Output the current bit
//                 i <= i + 1'b1;

//                 if (i == 8) begin
//                     spi_active <= 0; // End SPI transmission
//                     ss <= 1;  // Deactivate slave select
//                 end
//             end
//         end
//     end
// endmodule
