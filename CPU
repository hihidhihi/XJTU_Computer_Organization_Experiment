module CPU;
    reg CLK,CR,ALUC;
    wire [31:0]Instr,InstrID,PCN,PCI,PCO,JMP,Ins1,Ins2,Ins3,
               NPCID,RegWD,RegRD1,RegRD2,SigExtO,
               NPCEX,ALUA,RtEX,Imm32EX,NPC2EX,ALUB,ALUOutEX,RsO,HI,LO,NPC3I,
               NPC3O,NPC2MA,ALUOutMA,RtMA,InstrMA,DMemD,MemOutMA,NPCMA,
               MemOutWB,ALUOutWB,InstrWB;
               wire [31:0]IEX;
    wire DL,k,JR,U,CFEX,CFMA,CFWB,ZFEX,ZFMA,MemToReg,MemRW,Branch,ALUASrc,ALUBSrc,RegDst,RegWrite,Jump,Bjmp,PCSrc,HL,Signed,SignedMem;
    wire [3:0]ALUControl;
    wire [1:0]CS;
    wire [4:0]RegWA;
    wire [16:0]CtrlID,CtrlEX,CtrlMA,CtrlWB;
    assign k=(CtrlWB[16:15]==0)&CtrlWB[10];
    assign PCSrc=Bjmp|Jump;
    assign Bjmp=ZFMA&Branch;
    assign CS=CtrlMA[16:15];
    assign JR=CtrlEX[14];
    assign U=CtrlEX[13];
    assign ALUASrc=CtrlEX[12];
    assign SignedMem=CtrlMA[11];
    assign MemToReg=CtrlWB[10];
    assign MemRW=CtrlMA[9];
    assign Branch=CtrlMA[8];
    assign ALUBSrc=CtrlEX[7];
    assign RegDst=CtrlWB[6];
    assign RegWrite=CtrlWB[5]&(~CFWB);
    assign Jump=CtrlMA[4];
    assign ALUControl=CtrlEX[3:0];
    assign DMemD=MemRW?32'bz:RtMA;
    assign MemOutMA=MemRW?DMemD:NPCMA;
    initial begin
        CR<=1;
        CLK<=0;
        ALUC<=0;
        #5 CR<=0;
        forever #5 CLK=~CLK;
    end
    
    Hazard hzd(Instr,DL,CLK,Ins1,Ins2,Ins3,CR);
     
    //FI
    Reg PC(PCI,PCO,CLK,CR);
    IMem imem(PCO[5:0],Instr,Ins1,Ins2,Ins3);
    ADD4 add1(PCO,PCN,DL);
    MUX mux(PCN,JMP,PCSrc,PCI);
    
    Reg NPC1FD(PCN,NPCID,CLK,CR);
    Reg IRFD(DL?32'h0x00000000:Instr,InstrID,CLK,CR);
    
    
    //ID
    Controller CU(InstrID[31:26],InstrID[5:0],{Signed,HL,CtrlID});
    RegFile RF(CLK,RegWrite,InstrID[25:21],InstrID[20:16],RegWA,RegWD,RegRD1,RegRD2,CR,k);
    SigExt S(InstrID[15:0],HL,Signed,SigExtO);
    
    Reg CUIX(CtrlID,CtrlEX,CLK,CR);
    Reg NPC1IX(NPCID,NPCEX,CLK,CR);
    Reg Rs(RegRD1,RsO,CLK,CR);
    Reg RtIX(RegRD2,RtEX,CLK,CR);
    Reg Imm32(SigExtO,Imm32EX,CLK,CR);
    Reg IRIX(InstrID,IEX,CLK,CR);
    
    
    //EX
    ADD2 add2(NPCEX,Imm32EX,NPC2EX);
    MUX mux0(RsO,{27'b0,IEX[10:6]},ALUASrc,ALUA);
    MUX mux1(RtEX,Imm32EX,ALUBSrc,ALUB);
    MUX mux5({NPCEX[31:26],IEX[25:0]},RsO,JR,NPC3I);
    ALU alu(ALUOutEX,CFEX,ZFEX,ALUA,ALUB,ALUControl,ALUC,HI,LO,CR,U);
    
    Reg CUEA({CFEX,ZFEX,CtrlEX},{CFMA,ZFMA,CtrlMA},CLK,CR);
    Reg NPC3(NPC3I,NPC3O,CLK,CR);
    Reg NPC2(NPC2EX,NPC2MA,CLK,CR);
    Reg NPC1EA(NPCEX,NPCMA,CLK,CR);
    Reg ALUOutEA(ALUOutEX,ALUOutMA,CLK,CR);
    Reg RtEA(RtEX,RtMA,CLK,CR);
    Reg IREA(IEX,InstrMA,CLK,CR);
    //MA
    MUX4 mux4(NPC3O,NPC2MA,{Bjmp,Jump},JMP);
    RAM4k32 DMem(DMemD,ALUOutMA,CR,MemRW,CS,CLK,SignedMem);
    
    Reg CUMB({CFMA,CtrlMA},{CFWB,CtrlWB},CLK,CR);
    Reg MemOut(MemOutMA,MemOutWB,CLK,CR);
    Reg ALUOutMB(ALUOutMA,ALUOutWB,CLK,CR);
    Reg IRMB(InstrMA,InstrWB,CLK,CR);
    
    
    //WB
    MUX mux2(ALUOutWB,MemOutWB,MemToReg,RegWD);
    MUX mux3(InstrWB[15:11],InstrWB[20:16],RegDst,RegWA);
endmodule

module Hazard(Instr,DL,CLK,Ins1,Ins2,Ins3,CR);
    input [31:0]Instr,Ins1,Ins2,Ins3;
    input CLK,CR;
    output DL;
    assign DL=(i!=0);
    integer i;
    reg [4:0]r11,r12,r21,r22,r31,r32;
    always@(*)if(CR)begin i<=0;r11<=0;r12<=0;r21<=0;r22<=0;r31<=0;r32<=0;end
    always@(posedge CLK)begin
        if(i>0)i<=i-1;
        if(i==0)casez({Instr[31:26],Instr[5:0]})
            12'b00001zzzzzzz,
            12'b0001zzzzzzzz,
            12'b000000001000:i<=3;
            12'b1010zzzzzzzz,12'b0000000110zz:;
            12'b000000zzzzzz:if(Instr[15:11]==0);
                else if(Instr[15:11]==r11||Instr[15:11]==r12)i<=3;
                else if(Instr[15:11]==r21||Instr[15:11]==r22)i<=2;
                else if(Instr[15:11]==r21||Instr[15:11]==r22)i<=1;
            default:if(Instr[20:16]==0);
                else if(Instr[20:16]==r11||Instr[20:16]==r12)i<=3;
                else if(Instr[20:16]==r21||Instr[20:16]==r22)i<=2;
                else if(Instr[20:16]==r21||Instr[20:16]==r22)i<=1;
        endcase
    end
    always@(*)begin
        casez(Ins1[31:26])
            6'b000000,
            6'b1010zz,6'b0001zz:begin r11<=Ins1[25:21];r12<=Ins1[20:16];end
            6'b00001z:begin r11<=0;r12<=0;end
            default:begin r11<=Ins1[25:21];r12<=0;end
        endcase
        casez(Ins2[31:26])
            6'b000000,
            6'b1010zz,6'b0001zz:begin r21<=Ins1[25:21];r22<=Ins1[20:16];end
            6'b00001z:begin r21<=0;r22<=0;end
            default:begin r21<=Ins1[25:21];r22<=0;end
        endcase
        casez(Ins3[31:26])
            6'b000000,
            6'b1010zz,6'b0001zz:begin r31<=Ins1[25:21];r32<=Ins1[20:16];end
            6'b00001z:begin r31<=0;r32<=0;end
            default:begin r31<=Ins1[25:21];r32<=0;end
        endcase
    end
endmodule

module Reg(D,Q,CLK,CR);
    parameter N=32;
    input CLK,CR;
    input [N-1:0]D;
    output [N-1:0]Q;
    reg [N-1:0]Q;
    always@(posedge CLK or posedge CR)
        if(CR)Q<=0;
        else Q<=D;
endmodule

module IMem(A,RD,i1,i2,i3);
    input [5:0]A;
    output [31:0]RD,i1,i2,i3;
    reg [31:0] RAM[63:0];
    initial
    $readmemh("C:\\mips1",RAM);
    assign RD=RAM[A];
    assign i1=RAM[A+1];
    assign i2=RAM[A+2];
    assign i3=RAM[A+3];
endmodule

module ADD4(B,C,DL);
    input [31:0]B;
    input DL;
    output [31:0]C;
    assign C=DL?B:(B+1);
endmodule

module MUX(A,B,I,O);
    input [31:0]A,B;
    input I;
    output [31:0]O;
    assign O=I==0?A:B;
endmodule

module Controller(Op,Funct,Ctrl);
input[5:0]Op,Funct;
output [18:0]Ctrl;
wire[2:0]ALUOp;
wire [1:0]CS;
wire Signed,HL,JR,U,ALUASrc,MemToReg,MemRW,Branch,ALUSrc,RegDst,RegWrite,Jump,SignedMem;
wire [3:0]ALUControl;
MainDec M1(Op,MemToReg,MemRW,CS,Branch,ALUSrc,RegDst,RegWrite,Jump,ALUOp,HL,Signed,SignedMem);
ALUDec A(Funct,ALUOp,ALUControl,ALUASrc,JR);
UO uo({Op,Funct},U);
assign Ctrl={Signed,HL,CS,JR,U,ALUASrc,SignedMem,MemToReg,MemRW,Branch,ALUSrc,RegDst,RegWrite,Jump|JR,ALUControl};
endmodule

module MainDec(Op,MemToReg,MemRW,CS,Branch,ALUSrc,RegDst,RegWrite,Jump,ALUOp,HL,Signed,SignedMem);
input[5:0]Op;
output Signed,HL,MemToReg,MemRW,Branch,ALUSrc,RegDst,RegWrite,Jump,SignedMem;
output[2:0]ALUOp;
output [1:0]CS;
reg[15:0]Controls;
assign{CS,Signed,HL,SignedMem,RegWrite,RegDst,ALUSrc,Branch,MemRW,MemToReg,Jump,ALUOp}=Controls;
always@(*)
    case(Op)
        6'b000000:Controls<=15'b000001000000111;//R-type
        6'b000010:Controls<=15'b000000000001000;//j
        6'b000011:Controls<=15'b000001000011000;//jal
        6'b000100:Controls<=15'b001000001000001;//beq
        6'b000101:Controls<=15'b001000001000110;//bne
        6'b001000:Controls<=15'b001001110000000;//addi
        6'b001001:Controls<=15'b000001110000000;//addiu
        6'b001010:Controls<=15'b001001110000100;//slti
        6'b001011:Controls<=15'b000001110000100;//sltiu
        6'b001100:Controls<=15'b000001110000011;//andi
        6'b001101:Controls<=15'b000001110000010;//ori
        6'b001110:Controls<=15'b000001110000001;//xori
        6'b001111:Controls<=15'b000101110000000;//lui
        6'b100000:Controls<=15'b011011110110000;//lb
        6'b100100:Controls<=15'b011001110110000;//lbu
        6'b100001:Controls<=15'b101011110110000;//lh
        6'b100101:Controls<=15'b101001110110000;//lhu
        6'b100011:Controls<=15'b111011110110000;//lw
        6'b101000:Controls<=15'b011010010010000;//sb
        6'b101001:Controls<=15'b101010010010000;//sh
        6'b101011:Controls<=15'b111010010010000;//sw
        default:Controls<=13'bxxxxxxxxxxxxx;
    endcase
endmodule

module ALUDec(Funct,ALUOp,ALUControl,ALUASrc,JR);
input[5:0]Funct;
input[2:0]ALUOp;
output reg[3:0]ALUControl;
output reg ALUASrc;
output reg JR;
always@(*)
    case(ALUOp)
        3'b000:begin ALUControl<=4'b0100;ALUASrc<=0;JR<=0;end//+
        3'b001:begin ALUControl<=4'b0010;ALUASrc<=0;JR<=0;end//^
        3'b010:begin ALUControl<=4'b0001;ALUASrc<=0;JR<=0;end//|
        3'b011:begin ALUControl<=4'b0000;ALUASrc<=0;JR<=0;end//&
        3'b100:begin ALUControl<=4'b0110;ALUASrc<=0;JR<=0;end//>
        3'b101:;
        3'b110:begin ALUControl<=4'b1011;ALUASrc<=0;JR<=0;end//~^
        3'b111:begin
            case(Funct)
                6'b000000:ALUControl<=4'b0111;//sll
                6'b000010:ALUControl<=4'b1000;//srl
                6'b000011:ALUControl<=4'b1001;//sra
                6'b000100:ALUControl<=4'b0111;//sllv
                6'b000110:ALUControl<=4'b1000;//srlv
                6'b000111:ALUControl<=4'b1001;//srav
                6'b001000:ALUControl<=4'b0000;//jr
                6'b010000:ALUControl<=4'b1010;//mfhi
                6'b010010:ALUControl<=4'b1101;//mflo
                6'b011000:ALUControl<=4'b1100;//mult
                6'b011001:ALUControl<=4'b1100;//multu
                6'b011010:ALUControl<=4'b1110;//div
                6'b011011:ALUControl<=4'b1110;//divu
                6'b100000:ALUControl<=4'b0100;//add
                6'b100001:ALUControl<=4'b0100;//addu
                6'b100010:ALUControl<=4'b0101;//sub
                6'b100011:ALUControl<=4'b0101;//subu
                6'b100100:ALUControl<=4'b0000;//and
                6'b100101:ALUControl<=4'b0001;//or
                6'b100110:ALUControl<=4'b0010;//xor
                6'b100111:ALUControl<=4'b0011;//nor
                6'b101010:ALUControl<=4'b0110;//slt
                6'b101011:ALUControl<=4'b0110;//sltu
                default:ALUControl<=4'bxxxx;
            endcase
            case(Funct)
                6'b000000,6'b000010,6'b000011:ALUASrc<=1;
                default:ALUASrc<=0;
            endcase
            if(Funct==6'b001000)JR<=1;
            else JR<=0;
        end
        default:;
    endcase
endmodule

module UO(OpFunc,U);
    input [11:0]OpFunc;
    output reg U;
    always@(*)
        casez(OpFunc[11:6])
            12'b000000:case(OpFunc[5:0])
                6'b011001,6'b011011,6'b100001,6'b100011,6'b101011:U<=1;
                default:U<=0;
            endcase
            6'b001001,6'b001011:U<=1;
            default:U<=0;
        endcase
endmodule

`define DATA_WIDTH 32
module RegFile #(parameter ADDR_SIZE = 5)(CLK,WE3,RA1,RA2,WA3,WD3,RD1,RD2,reset,k);
input CLK,WE3,reset,k;
input [ADDR_SIZE-1:0]RA1,RA2,WA3;
input[`DATA_WIDTH-1:0]WD3;
output[`DATA_WIDTH-1:0]RD1,RD2;
reg[`DATA_WIDTH-1:0]rf[2**ADDR_SIZE-1:0];
integer i;
always@(reset)if(reset)for(i=0;i<2**ADDR_SIZE;i=i+1)rf[i]<=0;
always@(negedge CLK)if(k&~reset)rf[31]=WD3;
    else if(WE3&~reset)rf[WA3]=WD3;
assign RD1=rf[RA1];
assign RD2=rf[RA2];
endmodule

module SigExt(I,HL,Signed,O);
    input [15:0]I;
    input HL;
    input Signed;
    output [31:0]O;
    assign O=HL?{I,16'b0}:(Signed?(I[15]?{16'h0xffff,I}:{16'h0x0000,I}):{16'h0x0000,I});
endmodule

module ADD2(A,B,C);
    input [31:0]A,B;
    output reg[31:0]C;
    always@(*)C<=A+B;
endmodule

module ALU(F,CF,ZF,A,B,OP,C,HI,LO,CR,U);
    parameter SIZE=32;
    output reg [SIZE-1:0]F,HI,LO;
    output reg CF,ZF;
    input[SIZE-1:0]A,B;
    input[3:0]OP;
    input C,CR,U;
    parameter ALU_AND=4'b0000;
    parameter ALU_OR=4'b0001;
    parameter ALU_XOR=4'b0010;
    parameter ALU_NOR=4'b0011;
    parameter ALU_ADD=4'b0100;
    parameter ALU_SUB=4'b0101;
    parameter ALU_SLT=4'b0110;
    parameter ALU_SLL=4'b0111;
    parameter ALU_SRL=4'b1000;
    parameter ALU_SRA=4'b1001;
    parameter ALU_MFHI=4'b1010;
    parameter ALU_XNOR=4'b1011;
    parameter ALU_MULT=4'b1100;
    parameter ALU_MFLO=4'b1101;
    parameter ALU_DIV=4'b1110;
    parameter ALU_RESERVE=4'b1111;
    wire[SIZE-1:0]Fw;
    wire cf;
    wire U;
    always@(*)begin
        case(OP)
            ALU_AND:F<=A&B;
            ALU_OR:F<=A|B;
            ALU_XOR:F<=A^B;
            ALU_NOR:F<=~(A|B);
            ALU_XNOR:F<=A^B;
            ALU_SLT:F<=(U?A<B:$signed(A)<$signed(B))?32'h0x00000001:32'b0;
            ALU_SLL:F<=B<<A;
            ALU_SRL:F<=B>>A;
            ALU_SRA:F<=$signed(B)>>>A;
            ALU_MULT:begin F<=0;{HI,LO}<=U?A*B:A[31]^B[31]?~($unsigned(A[31]?(~A+1):A)*$unsigned(B[31]?(~B+1):B))+1:$unsigned(A[31]?(~A+1):A)*$unsigned(B[31]?(~B+1):B);end
            ALU_DIV:begin F<=0;LO<=U?A/B:A[31]^B[31]?~($unsigned(A[31]?(~A+1):A)/$unsigned(B[31]?(~B+1):B))+1:$unsigned(A[31]?(~A+1):A)/$unsigned(B[31]?(~B+1):B);
                HI<=U?A%B:A[31]^B[31]?~($unsigned(A[31]?(~A+1):A)%$unsigned(B[31]?(~B+1):B))+1:$unsigned(A[31]?(~A+1):A)%$unsigned(B[31]?(~B+1):B);end
            ALU_MFHI:F<=HI;
            ALU_MFLO:F<=LO;
            ALU_ADD,ALU_SUB:F<=Fw;
        endcase
        case(OP)
            ALU_ADD,ALU_SUB:CF<=U?0:cf;
            default:CF<=1'b0;
        endcase
        case(OP)
            ALU_XNOR:ZF<=F!=0?1'b1:1'b0;
            default:ZF<=F==0?1'b1:1'b0;
        endcase
        if(CR){HI,LO}<=0;
    end
    ADD add(Fw,cf,A,B,OP[0],C);
endmodule

module ADD(F,CF,A,B,EN,C);
    parameter N=32;
    output [N-1:0]F;
    output CF;
    input[N-1:0]A,B;
    input EN,C;
    wire [N-1:0]f,b;
    wire cf;
    assign b=EN?-B:B;
    //addr_32bit FA(f,cf,A,B,C);
    Look_ahead_add l(F,CF,A,b,C);
endmodule

module Look_ahead_add(S,CF,A,B,C_1);
    input [31:0]A,B;
    input C_1;
    output[31:0]S;
    output CF;
    wire [31:0]p,g;
    wire [32:1]c;
    wire [7:0]pp,gg;
    assign CF=c[31]^c[32];
    pg PG(p,g,A,B);
    addpc ADD(S,p,{c[31:1],C_1});
    BCLA bcla0(c[3:1],pp[0],gg[0],p[3:0],g[3:0],C_1);
    BCLA bcla1(c[7:5],pp[1],gg[1],p[7:4],g[7:4],c[4]);
    BCLA bcla2(c[11:9],pp[2],gg[2],p[11:8],g[11:8],c[8]);
    BCLA bcla3(c[15:13],pp[3],gg[3],p[15:12],g[15:12],c[12]);
    BCLA bcla4(c[19:17],pp[4],gg[4],p[19:16],g[19:16],c[16]);
    BCLA bcla5(c[23:21],pp[5],gg[5],p[23:20],g[23:20],c[20]);
    BCLA bcla6(c[27:25],pp[6],gg[6],p[27:24],g[27:24],c[24]);
    BCLA bcla7(c[31:29],pp[7],gg[7],p[31:28],g[31:28],c[28]);
    CLA cla1({c[16],c[12],c[8],c[4]},pp[3:0],gg[3:0],C_1);
    CLA cla2({c[32],c[28],c[24],c[20]},pp[7:4],gg[7:4],c[16]);
endmodule

module pg(p,g,a,b);
input [31:0]a,b;
output [31:0]p,g;
assign p=a^b;
assign g=a&b;
endmodule

module addpc(f,p,c);
input[31:0]p,c;
output[31:0]f;
assign f=p^c;
endmodule

module BCLA(c,pp,gg,p,g,c0);
input [3:0]p,g;
input c0;
output [3:1]c;
output pp,gg;
assign c[1]=g[0]|(p[0]&c0);
assign c[2]=g[1]|(p[1]&g[0])|(p[1]&p[0]&c0);
assign c[3]=g[2]|(p[2]&g[1])|(p[2]&p[1]&g[0])|(p[2]&p[1]&p[0]&c0);
assign pp=p[3]&p[2]&p[1]&p[0];
assign gg=g[3]|(p[3]&g[2])|(p[3]&p[2]&g[1])+(p[3]&p[2]&p[1]&g[0]);
endmodule

module CLA(c,p,g,c0);
input [3:0]p,g;
input c0;
output [4:1]c;
assign c[1]=g[0]|(p[0]&c0);
assign c[2]=g[1]|(p[1]&g[0])|(p[1]&p[0]&c0);
assign c[3]=g[2]|(p[2]&g[1])|(p[2]&p[1]&g[0])|(p[2]&p[1]&p[0]&c0);
assign c[4]=g[3]|(p[3]&g[2])|(p[3]&p[2]&g[1])|(p[3]&p[2]&p[1]&g[0])|(p[3]&p[2]&p[1]&p[0]&c0);
endmodule

module MUX4(A,B,I,O);
input [31:0]A,B;
input [1:0]I;
output [31:0]O;
assign O=I==1?A:B;
endmodule

module RAM4k32(DataO,Addr,Rst,R_W,CS,CLK,Signed);
    parameter Addr_Width=14;
    inout [31:0]DataO;
    input [Addr_Width-1:0]Addr;
    input Rst,R_W,CLK,Signed;
    input [1:0]CS;
    wire [3:0]CSb,CSw;
    wire [31:0]Data;
    wire [31:0]W,H0,H2,B0,B1,B2,B3;
    DEC dec(Addr[1:0],CS,CSb);
    Decoder24 d24(CS,CSw,Addr[Addr_Width-1:Addr_Width-2]);
    assign B0={32{{Addr[1:0],CS}==4'b0001}};
    assign B1={32{{Addr[1:0],CS}==4'b0101}};
    assign B2={32{{Addr[1:0],CS}==4'b1001}};
    assign B3={32{{Addr[1:0],CS}==4'b1101}};
    assign H0={32{{Addr[1:0],CS}==4'b0010}};
    assign H2={32{{Addr[1:0],CS}==4'b1010}};
    assign W={32{{Addr[1:0],CS}==4'b0011}};
    reg [31:0]Datai;
    always@(*)case({Addr[1:0],CS})
                4'b0001:Datai<=Signed?{{24{Data[7]}},Data[7:0]}:{24'b0,Data[7:0]};
                4'b0101:Datai<=Signed?{{24{Data[15]}},Data[15:8]}:{24'b0,Data[15:8]};
                4'b1001:Datai<=Signed?{{24{Data[23]}},Data[23:16]}:{24'b0,Data[23:16]};
                4'b1101:Datai<=Signed?{{24{Data[31]}},Data[31:24]}:{24'b0,Data[31:24]};
                4'b0010:Datai<=Signed?{{16{Data[15]}},Data[15:0]}:{16'b0,Data[15:0]};
                4'b1010:Datai<=Signed?{{16{Data[31]}},Data[31:16]}:{16'b0,Data[31:16]};
                4'b0011:Datai<=Data;
                default:;
    endcase
    assign DataO=R_W?Datai:32'bz;
    assign Data=R_W?
                32'bz:
                ((W&DataO)|
                 (H0&DataO)|
                 (H2&(DataO<<16))|
                 (B0&DataO)|
                 (B1&(DataO<<8))|
                 (B2&(DataO<<16))|
                 (B3&(DataO<<24)));
    RAM1k8 R0HH(Data[31:24],Addr[Addr_Width-3:2],Rst,R_W,CSw[0]&CSb[3],CLK),
           R0HL(Data[23:16],Addr[Addr_Width-3:2],Rst,R_W,CSw[0]&CSb[2],CLK),
           R0LH(Data[15:8],Addr[Addr_Width-3:2],Rst,R_W,CSw[0]&CSb[1],CLK),
           R0LL(Data[7:0],Addr[Addr_Width-3:2],Rst,R_W,CSw[0]&CSb[0],CLK),
           R1HH(Data[31:24],Addr[Addr_Width-3:2],Rst,R_W,CSw[1]&CSb[3],CLK),
           R1HL(Data[23:16],Addr[Addr_Width-3:2],Rst,R_W,CSw[1]&CSb[2],CLK),
           R1LH(Data[15:8],Addr[Addr_Width-3:2],Rst,R_W,CSw[1]&CSb[1],CLK),
           R1LL(Data[7:0],Addr[Addr_Width-3:2],Rst,R_W,CSw[1]&CSb[0],CLK),
           R2HH(Data[31:24],Addr[Addr_Width-3:2],Rst,R_W,CSw[2]&CSb[3],CLK),
           R2HL(Data[23:16],Addr[Addr_Width-3:2],Rst,R_W,CSw[2]&CSb[2],CLK),
           R2LH(Data[15:8],Addr[Addr_Width-3:2],Rst,R_W,CSw[2]&CSb[1],CLK),
           R2LL(Data[7:0],Addr[Addr_Width-3:2],Rst,R_W,CSw[2]&CSb[0],CLK),
           R3HH(Data[31:24],Addr[Addr_Width-3:2],Rst,R_W,CSw[3]&CSb[3],CLK),
           R3HL(Data[23:16],Addr[Addr_Width-3:2],Rst,R_W,CSw[3]&CSb[2],CLK),
           R3LH(Data[15:8],Addr[Addr_Width-3:2],Rst,R_W,CSw[3]&CSb[1],CLK),
           R3LL(Data[7:0],Addr[Addr_Width-3:2],Rst,R_W,CSw[3]&CSb[0],CLK);
endmodule

module DEC(Addr,CS,CS_i);
    input [1:0]CS,Addr;
    output reg[3:0]CS_i;
    always@(*)
        case(CS)
            0:CS_i=0;
            1:case(Addr)
                0:CS_i=4'b0001;
                1:CS_i=4'b0010;
                2:CS_i=4'b0100;
                3:CS_i=4'b1000;
                default:CS_i=0;
              endcase
            2:case(Addr)
                0:CS_i=4'b0011;
                2:CS_i=4'b1100;
                default:CS_i=0;
              endcase
            3:if(Addr==0)CS_i=4'b1111;
                else CS_i=0;
            default:CS_i=0;
        endcase
endmodule

module Decoder24(CS,out4,in2);
input [1:0]in2;
input [1:0]CS;
output reg[3:0]out4;
always@(*)begin
    if(CS!=0)case(in2)
        0:out4<=4'b0001;
        1:out4<=4'b0010;
        2:out4<=4'b0100;
        3:out4<=4'b1000;
    endcase
    else  out4<=4'b0000;
end
endmodule

module RAM1k8(Data,Addr,Rst,R_W,CS,CLK);
    parameter Addr_Width=10;
    parameter Data_Width=8;
    parameter SIZE=2**Addr_Width;
    inout [Data_Width-1:0]Data;
    input [Addr_Width-1:0]Addr;
    input Rst;
    input R_W;
    input CS;
    input CLK;
    integer i;
    reg [Data_Width-1:0]RAM[SIZE-1:0];
    assign Data=(CS&(~Rst)&R_W)?RAM[Addr]:8'bz;
    always @(negedge CLK)
        if(CS&(~Rst)&(~R_W))RAM[Addr]=Data;
    always @(*)if(Rst)begin
        for(i=0;i<SIZE;i=i+1)RAM[i]=0;
    end
endmodule
