`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:31:01 PM
// Design Name: 
// Module Name: RAM
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


module RAM(i_clk, i_rst, i_write_en, i_addr, i_write_data, o_read_data);
    input i_clk, i_rst, i_write_en;
    input[2:0] i_addr;
    input [7:0] i_write_data;
    output reg [7:0] o_read_data;

    reg [7:0] mem [0:7];

    always @ (posedge i_clk) begin
        if(i_rst) begin
            mem[0] <= 8'b0;
            mem[1] <= 8'b0;
            mem[2] <= 8'b0;
            mem[3] <= 8'b0;
            mem[4] <= 8'b0;
            mem[5] <= 8'b0;
            mem[6] <= 8'b0;
            mem[7] <= 8'b0;
        end
        else begin
            if(i_write_en)
                mem[i_addr] <= i_write_data;
            else
                o_read_data <= mem[i_addr];
        end
    end
endmodule