`timescale 1ns / 1ps

module register #(parameter N=2)(enable, funsel, load, Q_out);
input enable;
input [1:0] funsel;
input [N-1:0] load;
output reg [N-1:0] Q_out;

always @(enable, funsel) begin
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