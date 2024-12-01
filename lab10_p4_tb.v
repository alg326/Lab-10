`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 03:36:22 PM
// Design Name: 
// Module Name: r_multiplier_tb
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


module r_multiplier_tb;

    // Inputs
    reg clk;
    reg rst;
    reg [2:0] adr1_rom;
    reg [2:0] adr2_rom;
    reg [2:0] adr_ram;

    // Output
    wire [7:0] prod;

    // Instantiate the r_multiplier module
    r_multiplier dut (
        .clk(clk),
        .rst(rst),
        .adr1_rom(adr1_rom),
        .adr2_rom(adr2_rom),
        .adr_ram(adr_ram),
        .prod(prod)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Clock with 10ns period
    end

    // Test stimulus
    initial begin
        // Reset the system
        rst = 1;
        #10; 
        rst = 0;

        // Test Case 1: Multiply ROM[1] and ROM[2], store in RAM[0]
        adr1_rom = 3'd1; // ROM data at address 1 = 4'b1100
        adr2_rom = 3'd2; // ROM data at address 2 = 4'b0110
        adr_ram = 3'd0;  // RAM address to store result
        #100; // Wait for operations to complete
        
        rst = 1;
        #10; 
        rst = 0;

        // Test Case 2: Multiply ROM[3] and ROM[4], store in RAM[1]
        adr1_rom = 3'd3; // ROM data at address 3 = 4'b0111
        adr2_rom = 3'd4; // ROM data at address 4 = 4'b1000
        adr_ram = 3'd1;  // RAM address to store result
        #100; // Wait for operations to complete
        
        rst = 1;
        #10; 
        rst = 0;

        // Test Case 3: Multiply ROM[5] and ROM[6], store in RAM[2]
        adr1_rom = 3'd5; // ROM data at address 5 = 4'b0001
        adr2_rom = 3'd6; // ROM data at address 6 = 4'b1101
        adr_ram = 3'd2;  // RAM address to store result
        #100; // Wait for operations to complete

        // Finish simulation
        $stop;
    end

endmodule
