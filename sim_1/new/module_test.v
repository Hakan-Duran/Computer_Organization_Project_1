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
   
    ir my_ir (.clk(clk), .data(data), .enable(enable), .funsel(funsel), .lh(lh), .irout(irout));
    
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

module reg8_8_tb;

    reg clk;
    reg [7:0] load;
    reg [2:0] o1sel;
    reg [2:0] o2sel;
    reg [1:0] funsel;
    reg [3:0] rsel;
    reg [3:0] tsel;
    wire [7:0] o1;
    wire [7:0] o2;
   
    reg8_8 my_reg8_8 (.clk(clk), .load(load), .o1sel(o1sel), .o2sel(o2sel), .funsel(funsel), .rsel(rsel), .tsel(tsel), .o1(o1), .o2(o2));
    
    initial begin
        clk = 1;

        load=8'b10010101;

        rsel = 4'b0100;
        tsel = 4'b0001;

        funsel = 2'b00;
        
        o1sel = 3'b101;
        o2sel = 3'b011;

        #10;

        funsel = 2'b01;
        
        #10;
        
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

module alu_tb;

reg clk;
reg [7:0] A;
reg [7:0] B;
reg [3:0] Funsel;
wire [7:0] OutALU;
wire [3:0] Flag;

alu my_alu (.clk(clk), .A(A), .B(B), .Funsel(Funsel), .Flag(Flag), .OutALU(OutALU));

    initial begin
    clk = 1;
    A = 8'b01111111;
    B = 8'b00000000;
    Funsel = 4'b0000;   #10;
    Funsel = 4'b0001;   #10;
    Funsel = 4'b0010;   #10;
    Funsel = 4'b0011;   #10;
    Funsel = 4'b0100;   #10;
    Funsel = 4'b0101;   #10;
    Funsel = 4'b0110;   #10;
    Funsel = 4'b0111;   #10;
    Funsel = 4'b1000;   #10;
    Funsel = 4'b1001;   #10;
    Funsel = 4'b1010;   #10;
    Funsel = 4'b1011;   #10;
    Funsel = 4'b1100;   #10;
    Funsel = 4'b1101;   #10;
    Funsel = 4'b1110;   #10;
    Funsel = 4'b1111;   #10;

    A = 8'b10101010;
    B = 8'b10101010;
    Funsel = 4'b0000;   #10;
    Funsel = 4'b0001;   #10;
    Funsel = 4'b0010;   #10;
    Funsel = 4'b0011;   #10;
    Funsel = 4'b0100;   #10;
    Funsel = 4'b0101;   #10;
    Funsel = 4'b0110;   #10;
    Funsel = 4'b0111;   #10;
    Funsel = 4'b1000;   #10;
    Funsel = 4'b1001;   #10;
    Funsel = 4'b1010;   #10;
    Funsel = 4'b1011;   #10;
    Funsel = 4'b1100;   #10;
    Funsel = 4'b1101;   #10;
    Funsel = 4'b1110;   #10;
    Funsel = 4'b1111;   #10;

    A = 8'b11111111;
    B = 8'b01111111;
    Funsel = 4'b0000;   #10;
    Funsel = 4'b0001;   #10;
    Funsel = 4'b0010;   #10;
    Funsel = 4'b0011;   #10;
    Funsel = 4'b0100;   #10;
    Funsel = 4'b0101;   #10;
    Funsel = 4'b0110;   #10;
    Funsel = 4'b0111;   #10;
    Funsel = 4'b1000;   #10;
    Funsel = 4'b1001;   #10;
    Funsel = 4'b1010;   #10;
    Funsel = 4'b1011;   #10;
    Funsel = 4'b1100;   #10;
    Funsel = 4'b1101;   #10;
    Funsel = 4'b1110;   #10;
    Funsel = 4'b1111;   #10;

    $finish;
    end

    always #5 clk = ~clk ;
endmodule