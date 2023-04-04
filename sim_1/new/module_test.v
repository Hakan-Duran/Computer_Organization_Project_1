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


reg [7:0] A;
reg [7:0] B;
reg [3:0] Funsel;
wire [7:0] OutALU;
wire [3:0] Flag;

alu my_alu (.A(A), .B(B), .Funsel(Funsel), .Flag(Flag), .OutALU(OutALU));

    initial begin
 
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


endmodule



module system_test();
   reg [1:0] outasel;
   reg[1:0] outbsel;
   reg [1:0] funsel_IR;
   reg [1:0] funsel_arf;
   reg [1:0] funsel_rf;
   reg [3:0] funsel_alu;
   reg [3:0] regsel_rf;
   reg [3:0] regsel_arf;
   reg clock;
   reg wrMEM;
   reg csMEM;
   reg IR_enable;
   reg IR_lh;
   reg [1:0] MUXSelA;
   reg [1:0] MUXSelB;
   reg MUXSelC;
   reg [2:0]rf_o1sel;
   reg [2:0]rf_o2sel;
   reg [3:0]rf_tsel;



   wire[7:0] IR_out_MSBs;


    system sys1( outasel, outbsel, funsel_IR, funsel_arf, funsel_rf, funsel_alu,regsel_rf,regsel_arf,  clock,  wrMEM,  csMEM,  IR_enable,  IR_lh,  MUXSelA,  MUXSelB,  MUXSelC,  rf_o1sel,   rf_o2sel,   rf_tsel, IR_out_MSBs );



initial begin
    clock=0;
    outasel=2'b00;
    outbsel=2'b01;
    funsel_IR=2'b01;
    funsel_arf=2'b11;
    funsel_rf=2'b01;
    funsel_alu=4'b1010;
    regsel_rf=4'b0100;
    regsel_arf=4'b0010;
    wrMEM=1;
    csMEM=0;
    IR_enable=1;
    IR_lh=1;
    MUXSelA=2'b00;
    MUXSelB=2'b10;
    MUXSelC=1;
    rf_o1sel =3'b010;
    rf_o2sel =3'b110;
    rf_tsel=4'b1110;

end    

always #3.5 clock=~clock;
always #9.3 outasel=outasel+2'b01;
always #8.6 outbsel=outasel+2'b01;
always #7 funsel_IR = funsel_IR+2'b01;
always #6 funsel_arf=funsel_arf+2'b01;
always #6.2 funsel_rf=funsel_rf+2'b01;
always #4 funsel_alu = funsel_alu+4'b0001;
always #5 regsel_rf =regsel_rf + 4'b0001;
always #5.3 regsel_arf=regsel_arf+4'b0001;
always #4.4 wrMEM=~wrMEM;
// always #7.1 csMEM=~csMEM;
// always #3.9 IR_enable=~IR_enable;
always #5.8 IR_lh=~IR_lh;
always #5.2 MUXSelA=MUXSelA+2'b01;
always #4.8 MUXSelB=MUXSelB+2'b01;
always #5.2 MUXSelC=~MUXSelC;
always #4.7 rf_o1sel=rf_o1sel+3'b001;
always #5.7 rf_o2sel=rf_o2sel+3'b001;
always #6.4  rf_tsel=rf_tsel+4'b0001;



endmodule