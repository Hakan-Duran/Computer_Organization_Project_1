`timescale 1ns/1ps

module register_tb;
    
    parameter N = 8;
    reg clk;
    reg enable;
    reg [1:0] funsel;
    reg [N-1:0] load;
    wire [N-1:0] Q_out;
   
    register #(.N(N)) my_reg (.clk(clk), .enable(enable), .funsel(funsel), .load(load), .Q_out(Q_out));
    
    initial begin
        clk = 1;
        load=8'b10010101;
        
        enable = 1;
        funsel = 2'b00;
        
        #10;
        
        funsel = 2'b01;
        
        #10;
        
        funsel = 2'b10;
        
        #10;
        
        funsel = 2'b10;
        
        #10;

        funsel = 2'b10;
        
        #10;

        funsel = 2'b10;
        
        #10;
        
        $finish;
    end

    always #5 clk = ~clk ;

endmodule

module ir_tb;

    reg clk;
    reg [7:0] data;
    reg enable;
    reg [1:0] funsel;
    reg lh;
    wire [15:0] irout;
   
    ir my_ir (.clk(clk), .data(data), .enable(clk), .funsel(funsel), .lh(lh), .irout(irout));
    
    initial begin
        clk = 1;
        data=8'b10010101;
        
        enable = 1;
        funsel = 2'b00;
        
        #10;
        lh = 1;
        funsel = 2'b01;
        
        #10;
        data = 8'b00000001;
        lh = 0;
        
        funsel = 2'b01;
        
        #10;
        
        funsel = 2'b11;
        
        #10;

        funsel = 2'b11;

        #10;

        funsel = 2'b11;
        
        $finish;
    end

    always #5 clk = ~clk ;

endmodule