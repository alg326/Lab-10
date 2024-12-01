`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:35:19 PM
// Design Name: 
// Module Name: ROM
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


module ROM(ROM_data, ROM_addr);
output reg [3:0] ROM_data;
input [2:0] ROM_addr;
always @(ROM_addr) begin
        case (ROM_addr)
            3'd0: ROM_data = 4'b0000;
            3'd1: ROM_data = 4'b1100;
            3'd2: ROM_data = 4'b0110;
            3'd3: ROM_data = 4'b0111;
            3'd4: ROM_data = 4'b1000;
            3'd5: ROM_data = 4'b0001;
            3'd6: ROM_data = 4'b1101;
            3'd7: ROM_data = 4'b1110;
        endcase
	end
endmodule
