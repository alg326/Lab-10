`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:36:56 PM
// Design Name: 
// Module Name: ROM_tb
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


module ROM_tb;

    // Inputs
    reg [2:0] ROM_addr;

    // Output
    wire [3:0] ROM_data;

    // Instantiate the ROM module
    ROM dut (
        .ROM_data(ROM_data),
        .ROM_addr(ROM_addr)
    );

    // Test Procedure
    initial begin
        // Step 1: Apply different address values
        ROM_addr = 3'd0; #10;

        ROM_addr = 3'd1; #10;

        ROM_addr = 3'd2; #10;

        ROM_addr = 3'd3; #10;

        ROM_addr = 3'd4; #10;

        ROM_addr = 3'd5; #10;

        ROM_addr = 3'd6; #10;

        ROM_addr = 3'd7; #10;

        // Finish the simulation
        #10;
        $finish;
    end

endmodule

