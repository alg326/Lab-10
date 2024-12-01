`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 03:22:20 PM
// Design Name: 
// Module Name: r_multiplier
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


module r_multiplier(
    input clk, 
    input rst, 
    input [2:0] adr1_rom, 
    input [2:0] adr2_rom, 
    input [2:0] adr_ram, 
    output [7:0] prod
);

    // Internal signals
    wire [3:0] rom_data;          // Data from ROM
    wire [3:0] rf_data1, rf_data2; // Outputs from Register File
    wire [7:0] product;           // Output of Multiplier
    wire [7:0] ram_data;          // Output from RAM
    wire w_rf, w_ram;             // Write enables
    wire [2:0] cu_adr;            // Address from Control Unit
    wire DA, SA, SB;              // Register File control signals
    wire [2:0] cu_state;          // Control Unit state output
    
    // Instantiate ROM
    ROM rom_inst (
        .ROM_data(rom_data),
        .ROM_addr(cu_adr)
    );

    // Instantiate Register File
    RF rf_inst (
        .A(rf_data1),
        .B(rf_data2),
        .SA(SA),
        .SB(SB),
        .D(rom_data),
        .DA(DA),
        .W(w_rf),
        .rst(rst),
        .clk(clk)
    );

    // Instantiate Multiplier
    multiplier_comb multiplier_inst (
        .a(rf_data1),
        .b(rf_data2),
        .p(product)
    );

    // Instantiate RAM
    RAM ram_inst (
        .i_clk(clk),
        .i_rst(rst),
        .i_write_en(w_ram),
        .i_addr(adr_ram),
        .i_write_data(product),
        .o_read_data(ram_data)
    );

    // Instantiate Control Unit
    CU cu_inst (
        .clk(clk),
        .reset(rst),
        .adr1(adr1_rom),
        .adr2(adr2_rom),
        .w_rf(w_rf),
        .adr(cu_adr),
        .DA(DA),
        .SA(SA),
        .SB(SB),
        .st_out(cu_state),
        .w_ram(w_ram)
    );

    // Output the product
    assign prod = ram_data;

endmodule

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

module RF(A, B, SA, SB, D, DA, W, rst, clk);
	output [3:0]A; // A bus
	output [3:0]B; // B bus
	input SA; // Select A - A Address
	input SB; // Select B - B Address
	input [3:0]D; // Data input
	input DA; // Data destination address
	input W; // write enable
	input rst; // positive logic asynchronous reset
	input clk;
	
	wire [1:0]load_enable;
	wire [3:0]R00, R01;
	
	
	Decoder1to2 decoder (load_enable, DA, W);
	RegisterNbit reg00 (D,R00,load_enable[0], rst, clk); //D-in, R00-out
	RegisterNbit reg01 (D,R01,load_enable[1], rst, clk);
	Mux2to1Nbit muxA (A,R00, R01, SA);
	Mux2to1Nbit muxB (B,R00, R01,SB); 

endmodule

module RegisterNbit(D, Q,  L, R, clock);
	parameter N = 4; // number of bits
	output reg [N-1:0]Q; // registered output
	input [N-1:0]D; // data input
	input L; // load enable
	input R; // positive logic asynchronous reset
	input clock; // positive edge clock
	
	always @(posedge clock or posedge R) begin
		if(R)
			Q <= 0;
		else if(L)
			Q <= D;
		else
			Q <= Q;
	end
endmodule

module Decoder1to2(m, S, en);
	input S; // select
	input en; // enable (positive logic)
	output [1:0]m; // 32 minterms
	
	assign m[0] = ~S&en;
	assign m[1] = S&en;
	
endmodule

module Mux2to1Nbit(o, i1,i2, s);
   input [3:0] i1,i2;
   input  s;
   output reg  [3:0] o;
 
always @(s or i1 or i2)
begin
   case (s)
      1'b0 : o = i1;
      1'b1 : o = i2;
      default : o = 4'b0;
   endcase
end
endmodule

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

module ROM (ROM_data, ROM_addr);
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

module multiplier_comb(a,b,p);
    input [3:0] a,b;
    wire [3:0] m0;
    wire [4:0] m1;
    wire [5:0] m2;
    wire [6:0] m3;

    wire [7:0] s1,s2,s3;
    output [7:0] p;

    assign m0 = {4{a[0]}} & b[3:0];
    assign m1 = {4{a[1]}} & b[3:0];
    assign m2 = {4{a[2]}} & b[3:0];
    assign m3 = {4{a[3]}} & b[3:0];

    assign s1 = m0+(m1<<1);
    assign s2 = s1+(m2<<2);
    assign s3 = s2+(m3<<3);
    assign p = s3;
endmodule
