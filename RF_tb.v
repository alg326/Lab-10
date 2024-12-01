`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 02:48:17 PM
// Design Name: 
// Module Name: RF_tb
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

module RD_tb;
//Register File Signals
    wire [3:0] A, B;
    reg [3:0] data_in;
    reg rst_rf, clk_rf, write_en;
    reg SA,SB,DA;

RF register_file (
        .A(A),
        .B(B),
        .SA(SA),
        .SB(SB),
        .D(data_in),
        .DA(DA),
        .W(write_en),
        .rst(rst_rf),
        .clk(clk_rf)
    );

 // Clock Generation
    initial begin
        clk_rf = 0;
        forever #5 clk_rf = ~clk_rf; // 10ns clock period
    end


    initial begin
        
        rst_rf = 1;
        #10;
        
        rst_rf = 0; 
        
        DA = 0;
        data_in=4'b0001;
        write_en=1;
        #10;
        
        DA=1;
        data_in=4'b0011;
        write_en=1;
        #10;
        
        write_en=0;
        SA=0;
        SB=1;
        #20;
        $finish;
    end

endmodule