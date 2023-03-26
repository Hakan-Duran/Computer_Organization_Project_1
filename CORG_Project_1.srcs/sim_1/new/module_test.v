`timescale 1ns/1ps

module register_tb;
    
    parameter N = 8;
    reg clk;
    reg enable;
    reg [1:0] funsel;
    reg [N-1:0] load;
    wire [N-1:0] Q_out;
   
    register #(.N(N)) my_reg (.clk(clk), .enable(enable), .funsel(funsel), .load(load), .Q_out(Q_out));

    integer i,j,k;
    initial begin
        load=8'd1;
        clk=0;
        for (i = 1; i > -1; i = i - 1) begin
            for (j = 0; j < 2; j = j + 1) begin
                for (k=0;k<2;k=k+1) begin
                    // for (l = 0; l < 2**N; l = l + 1) begin
                    //     for (n = 0; n < N; n = n + 1) begin
                    //         load[n] = (l >> n) & 1'b1;
                    //     end
                        enable = i;
                        funsel[1] = j;
                        funsel[0] = k;

                        #125;
                    // end
                end
            end
        end
    end
    always #5 clk = ~clk;


        // enable = 1;
        // funsel = 2'b00;
        // load = 8'b00;
        // clk = 0;

        // #10;
        // load = 8'b01;
        // funsel = 2'b01;
        
        // #10;
        // load = 8'b11;
        // funsel = 2'b11;
        
        // #10;
        // load = 8'b10;
        // funsel = 2'b00;
        
        // #10;
        // load = 8'b11;
        // funsel = 2'b01;
        
        // #10;
        // load = 8'b11;
        // funsel = 2'b10;
        
        // #10;
        // load = 8'b10;
        // funsel = 2'b10;

        // #10;
        // load = 8'b10;
        // funsel = 2'b10;
        
        // #10;
        // load = 8'b10;
        // funsel = 2'b10;
        
        // #10;
        // enable = 0;
        
        // #10;
        // enable = 1;
        // #10 $finish;
//    end
endmodule