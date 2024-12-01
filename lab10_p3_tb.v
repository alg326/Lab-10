`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:50:25 PM
// Design Name: 
// Module Name: lab10_p3_tb
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


module CU_tb;
    // Control Unit Signals
    reg clk, reset;
    reg [2:0] adr1, adr2;
    wire w_rf;
    wire [2:0] adr;
    wire DA, SA, SB;
    wire [2:0] st_out;
    wire [2:0] w_ram;

    // Instantiate the Control Unit
    CU control_unit (
        .clk(clk),
        .reset(reset),
        .adr1(adr1),
        .adr2(adr2),
        .w_rf(w_rf),
        .adr(adr),
        .DA(DA),
        .SA(SA),
        .SB(SB),
        .st_out(st_out),
        .w_ram(w_ram)
    );
    
    // Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    initial begin
       
        reset = 1;    // Reset Control Unit
        #10;
        
         reset = 0; // Release reset

      adr1 = 3'b000;
       #10;
        adr2 = 3'b001;
        #100;


        reset = 1;    // Reset Control Unit
        #10;
  
         reset = 0; // Release reset
         
       adr1 = 3'b010;
       #10;
        adr2 = 3'b011;
        #100;
       
        #100; // Wait for Control Unit to complete execution
        $finish;
    end

endmodule
