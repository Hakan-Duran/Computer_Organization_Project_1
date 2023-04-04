`timescale 1ns / 1ps

module register #(parameter N=2)(clk, enable, funsel, load, Q_out);
input clk;
input enable;
input [1:0] funsel;
input [N-1:0] load;
output reg [N-1:0] Q_out;

always @(posedge clk) begin
    if (enable) begin
      case (funsel)
         2'b00 : Q_out <= {N{1'b0}} ;
         2'b01 : Q_out <= load ;
         2'b10 : Q_out <= Q_out - {{(N-1){1'b0}}, 1'b1} ;
         2'b11 : Q_out <= Q_out + {{(N-1){1'b0}}, 1'b1} ;
         default : Q_out <= load ;
      endcase
     end
end
endmodule

module ir (
   clk, data, enable, funsel, lh, irout
);

input clk;
input [7:0] data;
input [1:0] funsel;
input enable;
input lh;
output reg [15:0] irout;

always @(posedge clk) begin
   if (enable) begin
      case (funsel)
         2'b00 : irout <= 16'b0; 
         2'b01 : begin 
            if (!lh) irout[7:0] <= data ;
            else irout[15:8] <= data ;
         end
         2'b10 : irout <= irout - {15'b0, 1'b1} ;
         2'b11 : irout <= irout + {15'b0, 1'b1} ;
      endcase
   end
end
   
endmodule

module mux_2_1(
    input [1:0] Data,
    input [0:0] sel,
    output [0:0] C
    );
    
    assign C = (~sel & Data[0]) | (sel & Data[1]);
    
endmodule

module mux_4_1 (
   input [3:0] Data,
   input [1:0] sel,
   output [0:0] out
);

   wire a1, a2;

   mux_2_1 mu1 (.Data(Data[1:0]), .sel(sel[0]), .C(a1));
   mux_2_1 mu2 (.Data(Data[3:2]), .sel(sel[0]), .C(a2));

   mux_2_1 mu3 (.Data({a2, a1}), .sel(sel[1]), .C(out));
   
endmodule

module mux_8_1(
    input [7:0] Data,
    input [2:0] sel,
    output [0:0] out
);

    wire sel1;
    wire sel2;
    wire sel3;
    wire sel4;
    wire sel5;
    wire sel6;

    mux_2_1 m1 (.Data(Data[1:0]), .sel(sel[0]), .C(sel1));
    mux_2_1 m2 (.Data(Data[3:2]), .sel(sel[0]), .C(sel2));
    mux_2_1 m3 (.Data(Data[5:4]), .sel(sel[0]), .C(sel3));
    mux_2_1 m4 (.Data(Data[7:6]), .sel(sel[0]), .C(sel4));
    
    mux_2_1 m5 (.Data({sel2, sel1}), .sel(sel[1]), .C(sel5));
    mux_2_1 m6 (.Data({sel4, sel3}), .sel(sel[1]), .C(sel6));
    
    mux_2_1 m7 (.Data({sel6, sel5}), .sel(sel[2]), .C(out));

endmodule

module reg8_8 (
   clk, load, o1sel, o2sel, funsel, rsel, tsel, o1, o2
);
   
   input clk;
   input [7:0] load;
   input [2:0] o1sel;
   input [2:0] o2sel;
   input [1:0] funsel;
   input [3:0] rsel;
   input [3:0] tsel;
   output [7:0] o1;
   output [7:0] o2;

   wire [7:0] w0, w1, w2, w3, w4, w5, w6, w7 ;

   register#(8) r1 (.clk(clk), .enable(rsel[3]), .funsel(funsel), .load(load), .Q_out(w4)) ; 
   register#(8) r2 (.clk(clk), .enable(rsel[2]), .funsel(funsel), .load(load), .Q_out(w5)) ; 
   register#(8) r3 (.clk(clk), .enable(rsel[1]), .funsel(funsel), .load(load), .Q_out(w6)) ; 
   register#(8) r4 (.clk(clk), .enable(rsel[0]), .funsel(funsel), .load(load), .Q_out(w7)) ;

   register#(8) t1 (.clk(clk), .enable(tsel[3]), .funsel(funsel), .load(load), .Q_out(w0)) ; 
   register#(8) t2 (.clk(clk), .enable(tsel[2]), .funsel(funsel), .load(load), .Q_out(w1)) ; 
   register#(8) t3 (.clk(clk), .enable(tsel[1]), .funsel(funsel), .load(load), .Q_out(w2)) ; 
   register#(8) t4 (.clk(clk), .enable(tsel[0]), .funsel(funsel), .load(load), .Q_out(w3)) ; 

   mux_8_1 o10mux (.Data({w7[0],w6[0],w5[0],w4[0],w3[0],w2[0],w1[0],w0[0]}), .sel(o1sel), .out(o1[0]));
   mux_8_1 o11mux (.Data({w7[1],w6[1],w5[1],w4[1],w3[1],w2[1],w1[1],w0[1]}), .sel(o1sel), .out(o1[1]));
   mux_8_1 o12mux (.Data({w7[2],w6[2],w5[2],w4[2],w3[2],w2[2],w1[2],w0[2]}), .sel(o1sel), .out(o1[2]));
   mux_8_1 o13mux (.Data({w7[3],w6[3],w5[3],w4[3],w3[3],w2[3],w1[3],w0[3]}), .sel(o1sel), .out(o1[3]));
   mux_8_1 o14mux (.Data({w7[4],w6[4],w5[4],w4[4],w3[4],w2[4],w1[4],w0[4]}), .sel(o1sel), .out(o1[4]));
   mux_8_1 o15mux (.Data({w7[5],w6[5],w5[5],w4[5],w3[5],w2[5],w1[5],w0[5]}), .sel(o1sel), .out(o1[5]));
   mux_8_1 o16mux (.Data({w7[6],w6[6],w5[6],w4[6],w3[6],w2[6],w1[6],w0[6]}), .sel(o1sel), .out(o1[6]));
   mux_8_1 o17mux (.Data({w7[7],w6[7],w5[7],w4[7],w3[7],w2[7],w1[7],w0[7]}), .sel(o1sel), .out(o1[7]));

   mux_8_1 o20mux (.Data({w7[0],w6[0],w5[0],w4[0],w3[0],w2[0],w1[0],w0[0]}), .sel(o2sel), .out(o2[0]));
   mux_8_1 o21mux (.Data({w7[1],w6[1],w5[1],w4[1],w3[1],w2[1],w1[1],w0[1]}), .sel(o2sel), .out(o2[1]));
   mux_8_1 o22mux (.Data({w7[2],w6[2],w5[2],w4[2],w3[2],w2[2],w1[2],w0[2]}), .sel(o2sel), .out(o2[2]));
   mux_8_1 o23mux (.Data({w7[3],w6[3],w5[3],w4[3],w3[3],w2[3],w1[3],w0[3]}), .sel(o2sel), .out(o2[3]));
   mux_8_1 o24mux (.Data({w7[4],w6[4],w5[4],w4[4],w3[4],w2[4],w1[4],w0[4]}), .sel(o2sel), .out(o2[4]));
   mux_8_1 o25mux (.Data({w7[5],w6[5],w5[5],w4[5],w3[5],w2[5],w1[5],w0[5]}), .sel(o2sel), .out(o2[5]));
   mux_8_1 o26mux (.Data({w7[6],w6[6],w5[6],w4[6],w3[6],w2[6],w1[6],w0[6]}), .sel(o2sel), .out(o2[6]));
   mux_8_1 o27mux (.Data({w7[7],w6[7],w5[7],w4[7],w3[7],w2[7],w1[7],w0[7]}), .sel(o2sel), .out(o2[7]));

endmodule

module arf (
   clk, load, outasel, outbsel, funsel, rsel, outa, outb
);
   input clk;
   input [7:0] load;
   input [1:0] outasel;
   input [1:0] outbsel;
   input [1:0] funsel;
   input [3:0] rsel;
   output [7:0] outa;
   output [7:0] outb;

   wire [7:0] w0, w1, w2, w3;

   register#(8) ar (.clk(clk), .enable(rsel[3]), .funsel(funsel), .load(load), .Q_out(w0)) ; 
   register#(8) sp (.clk(clk), .enable(rsel[2]), .funsel(funsel), .load(load), .Q_out(w1)) ; 
   register#(8) pcp (.clk(clk), .enable(rsel[1]), .funsel(funsel), .load(load), .Q_out(w2)) ; 
   register#(8) pc (.clk(clk), .enable(rsel[0]), .funsel(funsel), .load(load), .Q_out(w3)) ;

   mux_4_1 a1 (.Data({w3[0],w2[0],w1[0],w0[0]}), .sel(outasel), .out(outa[0]));
   mux_4_1 a2 (.Data({w3[1],w2[1],w1[1],w0[1]}), .sel(outasel), .out(outa[1]));
   mux_4_1 a3 (.Data({w3[2],w2[2],w1[2],w0[2]}), .sel(outasel), .out(outa[2]));
   mux_4_1 a4 (.Data({w3[3],w2[3],w1[3],w0[3]}), .sel(outasel), .out(outa[3]));
   mux_4_1 a5 (.Data({w3[4],w2[0],w1[4],w0[4]}), .sel(outasel), .out(outa[4]));
   mux_4_1 a6 (.Data({w3[5],w2[5],w1[5],w0[5]}), .sel(outasel), .out(outa[5]));
   mux_4_1 a7 (.Data({w3[6],w2[6],w1[6],w0[6]}), .sel(outasel), .out(outa[6]));
   mux_4_1 a8 (.Data({w3[7],w2[7],w1[7],w0[7]}), .sel(outasel), .out(outa[7]));

   mux_4_1 b1 (.Data({w3[0],w2[0],w1[0],w0[0]}), .sel(outbsel), .out(outb[0]));
   mux_4_1 b2 (.Data({w3[1],w2[1],w1[1],w0[1]}), .sel(outbsel), .out(outb[1]));
   mux_4_1 b3 (.Data({w3[2],w2[2],w1[2],w0[2]}), .sel(outbsel), .out(outb[2]));
   mux_4_1 b4 (.Data({w3[3],w2[3],w1[3],w0[3]}), .sel(outbsel), .out(outb[3]));
   mux_4_1 b5 (.Data({w3[4],w2[0],w1[4],w0[4]}), .sel(outbsel), .out(outb[4]));
   mux_4_1 b6 (.Data({w3[5],w2[5],w1[5],w0[5]}), .sel(outbsel), .out(outb[5]));
   mux_4_1 b7 (.Data({w3[6],w2[6],w1[6],w0[6]}), .sel(outbsel), .out(outb[6]));
   mux_4_1 b8 (.Data({w3[7],w2[7],w1[7],w0[7]}), .sel(outbsel), .out(outb[7]));

endmodule

//emre's code ***********************************************

module flag_reg (
    clk,
    load,
    cin
);
    input clk;
    input [3:0] load;
    output cin;
    wire [3:0] out;
    register#(4) r (.clk(clk), .enable(1'b1), .funsel(2'b01), .load(load), .Q_out(out));

    assign cin = out[2];

endmodule


module alu (clk, A, B, Cin, Funsel, Flag, OutALU);
input clk;
input [7:0] A;
input [7:0] B;
input [3:0] Funsel;
input Cin;
output reg [7:0] OutALU;
output reg [3:0] Flag;

reg [8:0] out;

always @(*) begin

    case (Funsel)
        4'b0000 : begin 
            OutALU = A; 
            Flag[1] = OutALU[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] <= 1;
                end
            else begin
                Flag[3] <= 0;
                end
            end
        4'b0001 : begin
            OutALU = B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0010 : begin
            OutALU = ~A;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0011 : begin
            OutALU = ~B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0100 : begin
            out = A+B;
            OutALU = out[7:0];
            Flag[0] = (A[7]&B[7])^OutALU[7];
            Flag[1] = OutALU[7];
            Flag[2] = out[8];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0101 : begin
            out = A-B;
            OutALU = out[7:0];
            Flag[0] = (A[7]^B[7])&(A[7]^OutALU[7]);
            Flag[1] = OutALU[7];
            Flag[2] = out[8];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0110 : begin
            out = A+~B+9'b000000001;
            OutALU = out[7:0];
            Flag[0] = (A[7]^B[7])&(A[7]^OutALU[7]);
            Flag[1] = OutALU[7];
            Flag[2] = out[8];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0111 : begin
            OutALU = A&B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1000 : begin
            OutALU = A|B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1001 : begin
            OutALU = ~(A&B);
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1010 : begin
            OutALU = A^B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1011 : begin
            OutALU = A<<1;
            Flag[1] = OutALU[7];
            Flag[2] = A[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1100 : begin
            OutALU = A>>1;
            Flag[1] = OutALU[7];
            Flag[2] = A[0];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1101 : begin
            OutALU = A<<1;
            Flag[0] = A[7]^OutALU[7];
            Flag[1] = OutALU[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1110 : begin
            OutALU = A>>1;
            OutALU[7] = A[7]; //Should negativity control be implemented??
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1111 : begin
            OutALU = A>>1;
            OutALU[7] = A[0];
            Flag[1] = OutALU[7];
            Flag[2] = A[0];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        default : begin
            OutALU = 8'b00000000;
            Flag = 4'b0000;
        end
    endcase
end
endmodule


//last question **************************************
//given module for memory
module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration of the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule


module twoToOneMuxOf8bits(input selector, input[7:0] in0, input[7:0] in1, output reg[7:0] out);

always @(*) begin
   case (selector)
      1'b0 : out= in0;
      1'b1 : out= in1;
      default: assign out= in0;
   endcase
end
   
endmodule


module fourToOneMuxOf8bits(input[1:0] selector, input[7:0] in0, input[7:0] in1,input[7:0] in2,input[7:0] in3, output reg[7:0] out);

always @(*) begin
   case (selector)
      2'b00 :  out= in0;
      2'b01 :  out= in1;
      2'b10 :  out= in2;
      2'b11 :  out= in3;
      default: out= in0;
   endcase
end
   
endmodule


module system (
   input [1:0] outasel,
   input[1:0] outbsel,
   input [1:0] funsel_IR,
   input [1:0] funsel_arf,
   input [1:0] funsel_rf,
   input [3:0] funsel_alu,


   input [3:0] regsel_rf,
   input [3:0] regsel_arf,
   input clock,
   input wrMEM,
   input csMEM,
   input IR_enable,
   input IR_lh,
   input [1:0] MUXSelA,
   input [1:0] MUXSelB,
   input MUXSelC,
   input [2:0]rf_o1sel, 
   input [2:0]rf_o2sel, 
   input [3:0]rf_tsel,

   output wire [7:0] IR_out_MSBs //most significant bits of IR_out
);




wire [7:0] arf_outa;//arf -muxA
wire [7:0] arf_outb;//arf- memory adress
wire [7:0] outALU; //waiting for emre *******************
wire [7:0] MEMout; //memory - IR- muxA
wire [15:0] IR_out; //direct IR output
wire [7:0] IR_out_LSBs; //less significant bits of IR_out
wire [7:0] muxA_out; //muxA- rf
wire [7:0] muxB_out;//muxB-arf
wire [7:0] muxC_out;//muxC-alu
wire [7:0] rf_o1;//rf-muxC
wire [7:0] rf_o2;//rf-alu


arf ARF(clock,muxB_out, outasel, outbsel,funsel_arf,regsel_arf,arf_outa,arf_outb);
Memory MEMORY(arf_outb, outALU, wrMEM, csMEM, clock, MEMout);

ir IR(clock, MEMout, IR_enable,funsel_IR,IR_lh, IR_out); 
assign IR_out_MSBs =IR_out[15:8];
assign IR_out_LSBs=IR_out[7:0];


fourToOneMuxOf8bits muxA( MUXSelA, outALU, MEMout,  IR_out_LSBs, arf_outa, muxA_out);
fourToOneMuxOf8bits muxB(MUXSelB, outALU,MEMout,IR_out_LSBs, arf_outa, muxB_out);

reg8_8 Register_File(clock, muxA_out, rf_o1sel, rf_o2sel, funsel_rf, regsel_rf, rf_tsel, rf_o1, rf_o2);

twoToOneMuxOf8bits muxC(MUXSelC, rf_o1, arf_outa, muxC_out);
// alu ALU(muxC_out,rf_o2,funsel_alu,outALU); //order A , B , funsel, outALU***************


endmodule