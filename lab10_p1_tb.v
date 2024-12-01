`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:31:35 PM
// Design Name: 
// Module Name: RAM_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module RAM_tb;

    // Inputs
    reg i_clk;
    reg i_rst;
    reg i_write_en;
    reg [2:0] i_addr;
    reg [7:0] i_write_data;

    // Output
    wire [7:0] o_read_data;

    // Instantiate the RAM module
    RAM dut (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_write_en(i_write_en),
        .i_addr(i_addr),
        .i_write_data(i_write_data),
        .o_read_data(o_read_data)
    );

    // Clock generation
    initial begin
        i_clk = 0;
        forever #5 i_clk = ~i_clk; // 10ns clock period
    end

    // Test Procedure
    initial begin
        // Step 1: Apply Reset
        i_rst = 1;
        i_write_en = 0;
        i_addr = 3'b000;
        i_write_data = 8'b00000000;

        #10; // Hold reset for a few cycles
        i_rst = 0;

        // Step 2: Write Data to Memory
        i_write_en = 1;
        
        // Write 8'b10101010 to address 3'b000
        i_addr = 3'b000;
        i_write_data = 8'b10101010;
        #10;

        // Write 8'b11001100 to address 3'b001
        i_addr = 3'b001;
        i_write_data = 8'b11001100;
        #10;

        // Write 8'b11110000 to address 3'b010
        i_addr = 3'b010;
        i_write_data = 8'b11110000;
        #10;

        i_write_en = 0;

        // Step 3: Read Data from Memory
        i_addr = 3'b000; // Read from address 3'b000
        #10;

        i_addr = 3'b001; // Read from address 3'b001
        #10;

        i_addr = 3'b010; // Read from address 3'b010
        #10;

        // Step 4: Reset Memory
        i_rst = 1;
        #10;
        i_rst = 0;

        // Verify memory is reset
        i_addr = 3'b000; // Read from address 3'b000 after reset
        #10;

        i_addr = 3'b001; // Read from address 3'b001 after reset
        #10;

        // Finish simulation
        #20;
        $finish;
    end

endmodule
