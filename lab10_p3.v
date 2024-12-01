`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:48:24 PM
// Design Name: 
// Module Name: lab10_p3
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

module CU ( 
  input clk, reset,
  input [2:0] adr1,
  input [2:0] adr2,
  output reg w_rf,
  output reg [2:0] adr,
  output reg DA,SA,SB,
  output reg [2:0] st_out,
  //output reg done,
  output reg [2:0] w_ram
);
    parameter S0_idle = 0 , S1_send_adr1 = 1 , S2_send_adr2 = 2 ,S3_multiply = 3 ,S4_write_ram = 4,S5_read_ram=5 ;
    reg [2:0] PS,NS ;
    always@(posedge clk or posedge reset)
        begin
            if(reset)
                PS <= S0_idle;   
            else    
                PS <= NS ;
        end
    always@(*)
        begin 
            
            case(PS)
				S0_idle:begin
				NS = S1_send_adr1;
                w_rf <=1'b1;
                w_ram <=1'b1;
                st_out <= 3'b000;
				end

				S1_send_adr1:begin	
				w_rf <=1'b1;
				adr<=adr1;
				DA <=1'b0;
				SA <=1'b0;
				SB <=1'b1;
				st_out <= 3'b001;
				NS = S2_send_adr2;
				end
				
				S2_send_adr2:begin
				w_rf <=1'b1;
				adr<=adr2;
				NS = S3_multiply;
				DA <=1'b1;
				SA <=1'b0;
				SB <=1'b1;
                st_out <= 3'b010;
			    end

		        S3_multiply: begin
				NS = S4_write_ram;
                st_out <= 3'b011;
				w_ram<=1;
			    end

                S4_write_ram: begin
                st_out <= 3'b100;
				NS = S5_read_ram;
				end

				S5_read_ram: begin
                w_ram<=0;
                //done <=1;
                st_out <= 3'b101;
				if(!reset) begin
				    NS = S5_read_ram;
				end
				else begin
				    NS = S0_idle;
				end
            end
        endcase
    end
endmodule

//module RF(A, B, SA, SB, D, DA, W, rst, clk);
//	output [3:0]A; // A bus
//	output [3:0]B; // B bus
//	input SA; // Select A - A Address
//	input SB; // Select B - B Address
//	input [3:0]D; // Data input
//	input DA; // Data destination address
//	input W; // write enable
//	input rst; // positive logic asynchronous reset
//	input clk;
	
//	wire [1:0]load_enable;
//	wire [3:0]R00, R01;
	
	
//	Decoder1to2 decoder (load_enable, DA, W);
//	RegisterNbit reg00 (D,R00,load_enable[0], rst, clk); //D-in, R00-out
//	RegisterNbit reg01 (D,R01,load_enable[1], rst, clk);
//	Mux2to1Nbit muxA (A,R00, R01, SA);
//	Mux2to1Nbit muxB (B,R00, R01,SB); 

//endmodule

//module RegisterNbit(D, Q,  L, R, clock);
//	parameter N = 4; // number of bits
//	output reg [N-1:0]Q; // registered output
//	input [N-1:0]D; // data input
//	input L; // load enable
//	input R; // positive logic asynchronous reset
//	input clock; // positive edge clock
	
//	always @(posedge clock or posedge R) begin
//		if(R)
//			Q <= 0;
//		else if(L)
//			Q <= D;
//		else
//			Q <= Q;
//	end
//endmodule

//module Decoder1to2(m, S, en);
//	input S; // select
//	input en; // enable (positive logic)
//	output [1:0]m; // 32 minterms
	
//	assign m[0] = ~S&en;
//	assign m[1] = S&en;
	
//endmodule

//module Mux2to1Nbit(o, i1,i2, s);
//   input [3:0] i1,i2;
//   input  s;
//   output reg  [3:0] o;
 
//always @(s or i1 or i2)
//begin
//   case (s)
//      1'b0 : o = i1;
//      1'b1 : o = i2;
//      default : o = 4'b0;
//   endcase
//end
//endmodule