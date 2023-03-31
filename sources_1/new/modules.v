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