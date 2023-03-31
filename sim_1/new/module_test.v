`timescale 1ns/1ps

module register_tb;
    
    parameter N = 8;
    reg enable;
    reg [1:0] funsel;
    reg [N-1:0] load;
    wire [N-1:0] Q_out;
   
    register #(.N(N)) my_reg (.enable(enable), .funsel(funsel), .load(load), .Q_out(Q_out));
    
    initial begin
        load=8'b10010101;
        
        enable = 1;
        funsel = 2'b00;
        
        #10;
        
        funsel = 2'b01;
        
        #10;
        
        funsel = 2'b10;
        
        #10;
        
        funsel = 2'b00;
        
        #10;
        
        $finish;
    end

endmodule