`timescale 1ns/1ps

module register_tb;
    
    parameter N = 8;
    reg clk;
    reg enable;
    reg [1:0] funsel;
    reg [N-1:0] load;
    wire [N-1:0] Q_out;

    register #(.N(N)) my_reg (.clk(clk), .enable(enable), .funsel(funsel), .load(load), .Q_out(Q_out));

    always #5 clk = ~clk;

    initial begin
        enable = 1;
        funsel = 2'b00;
        load = 8'b00;
        clk = 0;

        #10;
        load = 8'b01;
        funsel = 2'b01;
        
        #10;
        load = 8'b11;
        funsel = 2'b11;
        
        #10;
        load = 8'b10;
        funsel = 2'b00;
        
        #10;
        load = 8'b11;
        funsel = 2'b01;
        
        #10;
        load = 8'b11;
        funsel = 2'b10;
        
        #10;
        load = 8'b10;
        funsel = 2'b10;

        #10;
        load = 8'b10;
        funsel = 2'b10;
        
        #10;
        load = 8'b10;
        funsel = 2'b10;
        
        #10;
        enable = 0;
        
        #10;
        enable = 1;
        #10 $finish;
    end
endmodule