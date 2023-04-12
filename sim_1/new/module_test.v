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

        funsel = 2'b11;
        
        #10;
        
        funsel = 2'b11;
        
        #10;
        enable = 0;
        funsel = 2'b00;
        
        #10;
        load = 8'b11111100;
        funsel = 2'b10;
        
        #10;
        enable = 1;
        funsel = 2'b01;
        
        #10;
        funsel = 2'b11;
        
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
        data = 8'b00000010;
        lh = 0;
        
        funsel = 2'b01;
        
        #10;
        
        funsel = 2'b11;
        
        #10;

        funsel = 2'b11;

        #10;

        funsel = 2'b11;
        
        #10;
        data = 8'b00001000;
        lh = 1;
        funsel = 2'b01;
        
        #10;
        funsel = 2'b10;
        
        #20;
        
        funsel = 2'b00;
        
        #10;
        
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
        
        #10;
        funsel = 2'b00;
        
        #10;
        funsel = 2'b10;
        
        #20;
        
        $finish;

    end

    always #5 clk = ~clk ;

endmodule

module arf_tb();

   reg clk;
   reg[7:0] load;
   reg[1:0] outasel;
   reg[1:0] outbsel;
   reg[1:0] funsel;
   reg[3:0] rsel;
   wire[7:0] outa;
   wire[7:0] outb;

        arf ARF(clk, load, outasel, outbsel, funsel, rsel, outa, outb);

    initial begin
        clk=0;   outasel=2'b00; outbsel=2'b01;
        rsel=4'b 1111; funsel=2'b00; #10;

        load = 8'b00101010; #500;
        load = 8'b11101000; #500;




    end

    always #5 clk=~clk;
    always #100 rsel=rsel+4'b0001;
    always #103 funsel= funsel+2'b01;
    always #104 outasel=outasel+2'b01; 
    always #105 outbsel=outbsel+2'b01;
    



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

`timescale 1ns / 1ps



////this part was our modified test bench, if you want you can also use it :)
// module Project1Test();
//     //Input Registers of ALUSystem
//     reg[2:0] RF_O1Sel; 
//     reg[2:0] RF_O2Sel; 
//     reg[1:0] RF_FunSel;
//     reg[3:0] RF_RSel;
//     reg[3:0] RF_TSel;
//     reg[3:0] ALU_FunSel;
//     reg[1:0] ARF_OutASel; 
//     reg[1:0] ARF_OutBSel; 
//     reg[1:0] ARF_FunSel;
//     reg[3:0] ARF_RSel;
//     reg      IR_LH;
//     reg      IR_Enable;
//     reg[1:0]      IR_Funsel;
//     reg      Mem_WR;
//     reg      Mem_CS;
//     reg[1:0] MuxASel;
//     reg[1:0] MuxBSel;
//     reg MuxCSel;
//     reg      Clock;

// //outputs
//     wire [7:0] AOut;//rf-muxC
//     wire [7:0] BOut;//rf-alu
//     wire [7:0] ALUOut;
//     wire [3:0] ALUOutFlag;
//     wire [7:0] ARF_AOut;//arf -muxA
//     wire [7:0] Address;//arf- memory adress
//     wire [7:0] MemoryOut; //memory - IR- muxA
//     wire [7:0] MuxAOut; //muxA- rf
//     wire [7:0] MuxBOut;//muxB-arf
//     wire [7:0] MuxCOut;//muxC-alu
//     wire [15:0] IROut; //direct IR output
// //outputs will be commented later

    
//     //Test Bench Connection of ALU System
//     ALU_System _ALUSystem(
//     .RF_OutASel(RF_O1Sel), 
//     .RF_OutBSel(RF_O2Sel), 
//     .RF_FunSel(RF_FunSel),
//     .RF_RSel(RF_RSel),
//     .RF_TSel(RF_TSel),
//     .ALU_FunSel(ALU_FunSel),
//     .ARF_OutCSel(ARF_OutASel), 
//     .ARF_OutDSel(ARF_OutBSel), 
//     .ARF_FunSel(ARF_FunSel),
//     .ARF_RegSel(ARF_RSel),
//     .IR_LH(IR_LH),
//     .IR_Enable(IR_Enable),
//     .IR_Funsel(IR_Funsel),
//     .Mem_WR(Mem_WR),
//     .Mem_CS(Mem_CS),
//     .MuxASel(MuxASel),
//     .MuxBSel(MuxBSel),
//     .MuxCSel(MuxCSel),
//     .Clock(Clock),
//     //outputs
//     .AOut(AOut),
//     .BOut(BOut),
//     .ALUOut(ALUOut),
//     .ALUOutFlag(ALUOutFlag),
//     .ARF_AOut(ARF_AOut),
//     .Address(Address),
//     .MemoryOut(MemoryOut),
//     .MuxAOut(MuxAOut),
//     .MuxBOut(MuxBOut),
//     .MuxCOut(MuxCOut),
//     . IROut( IROut)
//     //this part will be commented later

//     );
    
//     //Test Vector Variables
//     reg [41:0] VectorNum, Errors, TotalLine; 
//     reg [41:0] TestVectors[3:0];
//     reg Reset, Operation;
//     initial begin
//         Reset = 0;
//     end
//     //Clock Signal Generation
//     always 
//     begin
//         Clock = 1; #5; Clock = 0; #5; // 10ns period
//     end
    
//     //Read Test Bench Values
//     initial begin
//         $readmemb("TestBench.mem", TestVectors); // Read vectors
//         VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
//     end
    
//     // Apply test vectors on rising edge of clock
//     always @(posedge Clock)
//     begin
//         #1; 
//         {Operation, RF_O1Sel, RF_O2Sel, RF_FunSel, 
//         RF_RSel, RF_TSel, ALU_FunSel, ARF_OutASel, ARF_OutBSel, 
//         ARF_FunSel, ARF_RSel, IR_LH, IR_Enable, IR_Funsel, 
//         Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
//     end
    
//     // Check results on falling edge of clk
//     always @(negedge Clock)
//         if (~Reset) // skip during reset
//         begin
//             $display("Input Values:");
//             $display("Operation: %b", Operation);
//             $display("Register File: O1Sel: %b, O2Sel: %b, FunSel: %b, RSel: %b, TSel: %b", RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel);            
//             $display("ALU FunSel: %b", ALU_FunSel);
//             $display("Addres Register File: OutASel: %b, OutBSel: %b, FunSel: %b, Regsel: %b", ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel);            
//             $display("Instruction Register: LH: %b, Enable: %b, FunSel: %b", IR_LH, IR_Enable, IR_Funsel);            
//             $display("Memory: WR: %b, CS: %b", Mem_WR, Mem_CS);
//             $display("MuxASel: %b, MuxBSel: %b, MuxCSel: %b", MuxASel, MuxBSel, MuxCSel);
            
//             $display("");
//             $display("Output Values:");
//             $display("Register File: AOut: %b, BOut: %b", _ALUSystem.AOut, _ALUSystem.BOut);            
//             $display("ALUOut: %b, ALUOutFlag: %b, ALUOutFlags: Z:%b, C:%b, N:%b, O:%b,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
//             $display("Address Register File: AOut: %b, BOut (Address): %b", _ALUSystem.AOut, _ALUSystem.Address);            
//             $display("Memory Out: %b", _ALUSystem.MemoryOut);            
//             $display("Instruction Register: IROut: %b", _ALUSystem.IROut);            
//             $display("MuxAOut: %b, MuxBOut: %b, MuxCOut: %b", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
//             // increment array index and read next testvector
//             VectorNum = VectorNum + 1;
//             if (TestVectors[VectorNum] === 42'bx)
//             begin
//                 $display("%d tests completed.",
//                 VectorNum);
//                 $finish; // End simulation
//             end
//         end
// endmodule







//this is your test bench without any modification
module Project1Test();
    //Input Registers of ALUSystem
    reg[2:0] RF_O1Sel; 
    reg[2:0] RF_O2Sel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RSel;
    reg[3:0] RF_TSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutASel; 
    reg[1:0] ARF_OutBSel; 
    reg[1:0] ARF_FunSel;
    reg[3:0] ARF_RSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0]      IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg MuxCSel;
    reg      Clock;
    
    //Test Bench Connection of ALU System
    ALU_System _ALUSystem(
    .RF_OutASel(RF_O1Sel), 
    .RF_OutBSel(RF_O2Sel), 
    .RF_FunSel(RF_FunSel),
    .RF_RSel(RF_RSel),
    .RF_TSel(RF_TSel),
    .ALU_FunSel(ALU_FunSel),
    .ARF_OutCSel(ARF_OutASel), 
    .ARF_OutDSel(ARF_OutBSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_Funsel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock)
    );
    
    //Test Vector Variables
    reg [41:0] VectorNum, Errors, TotalLine; 
    reg [41:0] TestVectors[3:0];
    reg Reset, Operation;
    initial begin
        Reset = 0;
    end
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //Read Test Bench Values
    initial begin
        $readmemb("TestBench.mem", TestVectors); // Read vectors
        VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
    end
    
    // Apply test vectors on rising edge of clock
    always @(posedge Clock)
    begin
        #1; 
        {Operation, RF_O1Sel, RF_O2Sel, RF_FunSel, 
        RF_RSel, RF_TSel, ALU_FunSel, ARF_OutASel, ARF_OutBSel, 
        ARF_FunSel, ARF_RSel, IR_LH, IR_Enable, IR_Funsel, 
        Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
    end
    
    // Check results on falling edge of clk
    always @(negedge Clock)
        if (~Reset) // skip during reset
        begin
            $display("Input Values:");
            $display("Operation: %d", Operation);
            $display("Register File: O1Sel: %d, O2Sel: %d, FunSel: %d, RSel: %d, TSel: %d", RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel);            
            $display("ALU FunSel: %d", ALU_FunSel);
            $display("Addres Register File: OutASel: %d, OutBSel: %d, FunSel: %d, Regsel: %d", ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel);            
            $display("Instruction Register: LH: %d, Enable: %d, FunSel: %d", IR_LH, IR_Enable, IR_Funsel);            
            $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
            $display("MuxASel: %d, MuxBSel: %d, MuxCSel: %d", MuxASel, MuxBSel, MuxCSel);
            
            $display("");
            $display("Output Values:");
            $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
            $display("ALUOut: %d, ALUOutFlag: %d, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
            $display("Address Register File: AOut: %d, BOut (Address): %d", _ALUSystem.AOut, _ALUSystem.Address);            
            $display("Memory Out: %d", _ALUSystem.MemoryOut);            
            $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
            $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
            // increment array index and read next testvector
            VectorNum = VectorNum + 1;
            if (TestVectors[VectorNum] === 42'bx)
            begin
                $display("%d tests completed.",
                VectorNum);
                $finish; // End simulation
            end
        end
endmodule

