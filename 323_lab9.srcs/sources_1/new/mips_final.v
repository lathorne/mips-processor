`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/20/2018 01:09:26 PM
// Design Name: 
// Module Name: mips_forwarding_pipeline
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mips_final(clk, PC, Instruction, ALUResult, dWriteData, WriteBackData);
    
    input clk;
    output [31:0] PC, Instruction, ALUResult, dWriteData, WriteBackData;
    
    //stalling wires
    wire halt_pc, insert_ex_noop;
    
    wire [1:0] jump;
    reg [1:0] jump_ID;
    reg [31:0] jumpAddress_ID;
    
    //outputs of register file
    reg [31:0] Data1, Data2;
    reg [31:0] RF [31:0]; 
    
     //Mem stage components
       wire PCSrc_MEM; 
       reg [31:0] data_output_MEM, MemtoReg_MEM, ALUOut_MEM;
       reg RegWrite_MEM;
       reg [5:0] WriteReg_MEM;    
     
    //WB components
    //using MEM output for WB stage  
    
    //registers to pass Rd, Rs, Rt
    reg [4:0] ID_Rd, ID_Rs, ID_Rt, EX_Rd, EX_Rs, EX_Rt, MEM_Rd, MEM_Rs, MEM_Rt; 
    
    //control wires for forwarding signals
    wire [1:0] forwardA, forwardB;
    wire forwardC; 
    
    //IF///////////////////////////////////////////////////////////////////////////////
     //IF registers
       reg [31:0] instruction_reg_IF, pc_reg_IF, pc_reg;
       wire [9:0] pc_addr;
       assign pc_addr = pc_reg[11:2]; //this should come straight out of the PC
       
    //initialize to zero
      initial begin
          instruction_reg_IF <= 0;
          pc_reg_IF <= 0;
          pc_reg <= 0;
      end
    
    reg [31:0] branch_target_EX; 
      
    //PC/////////////////  //jump = 1 rgular jump, jump = 2 jump and link, jump = 3 jr, else we ain't jumpin
    always@(posedge clk) begin
        if(halt_pc)
            pc_reg <= pc_reg; //added for stalling
        else if(PCSrc_MEM)
            pc_reg <= branch_target_EX; //comes from MEM stage
        else if(jump_ID == 3)
            pc_reg <= Data1;//jump from register 
        else if(jump_ID == 1 || jump_ID == 2)
            pc_reg <= jumpAddress_ID;
        else
            pc_reg <= PC + 4;
    end
    
    always@(posedge clk)
        if(halt_pc == 0)
            pc_reg_IF <= pc_reg + 4;
    
    // memory declaration (two dimensional array declaration 128 deep by 8 bits wide)
    reg [31:0] i_mem[1023:0];
    reg [31:0] d_mem[1023:0];
    
    // memory and PC initialization 
    initial begin
     $readmemh("final_instruction_memory.txt", i_mem);
     $readmemh("final_data_memory.txt", d_mem);
    end
      
    // Synchronous read (Data appears after clock edge)
    always@(posedge clk) begin
      if (pc_addr >= 0 && pc_addr <= 32'h0000007f) // decode address (this is instruction), address is the PC value
        if(halt_pc == 0) //janky fix - this might be an issue in future labs
            instruction_reg_IF <= i_mem[pc_addr]; 
    end
    //End IF///////////////////////////////////////////////////////////////////////////
    
    //ID///////////////////////////////////////////////////////////////////////////////
    
    //ID components
        wire [31:0] extend;
        wire regDst;
        reg ALUSrc_ID, Branch_ID, BranchBNE_ID, MemWrite_ID, MemRead_ID, RegWrite_ID, MemtoReg_ID;
        reg [31:0] instructionID, instructionEX, instructionMEM;
        reg [3:0] ALUCtrl_ID;
        reg [31:0] pc_reg_ID, extend_ID;
        reg [5:0] WriteReg_ID;
        
    //creating register file////////////////////////////////////////////////////
        
        wire [4:0] Read1, Read2, WriteReg;
        wire [31:0] WriteData;
        
        integer i;
                initial
                   for(i = 0; i < 32; i=i+1)
                        RF[i] = 0;
            
                //inputs of register file
        
       
        
        assign Read1 = instruction_reg_IF[25:21];
        assign Read2 = instruction_reg_IF[20:16];
    
//        assign WriteReg = regDst ? instruction_reg_IF [15:11] : instruction_reg_IF [20:16];
        
        // synchronous write and read with "write first" mode
          always@(posedge clk) begin
            // Default read: read old values
                Data1 <= RF[Read1];
                Data2 <= RF[Read2];
                if (RegWrite_MEM) begin
                  RF[WriteReg_MEM] <= WriteData; //writereg_MEM is the register address to write to
                  // If reading same register we are writing, return new data
                  if (Read1 == WriteReg_MEM)
                    Data1 <= WriteData;
                  if (Read2 == WriteReg_MEM)
                    Data2 <= WriteData;
                end
                
                if(jump == 2) //store the link address
                    RF[5'b11111] <=  pc_reg_IF; //this should be the PC plu 4, but we might need add 4 to it
                    
                if(insert_ex_noop)begin
                    Data1 <= 0;
                    Data2 <= 0;
                    end
              end
        //end of register file//////////////////////////////////////////////////////
    
        wire ZeroExt, MemtoReg, MemWrite, RegWrite, ALUSrc, MemRead;
        
        //sign/zero extend logic
        assign extend = ZeroExt ? {{16{0}}, instruction_reg_IF[15:0]}: //if zeroext is asserted, extend by all zeros, otherwise sign extend
                            {{16{instruction_reg_IF[15]}}, instruction_reg_IF[15:0]}; 
        
        //CONTROL//////////////////////////////////////////////////////////////////////
        //ALU Control signal assignment here////////////////////////////////////////////////////////////////////////
            wire [5:0] opcode, funct, arithmetic;
            wire branch, branchBNE;
            wire [3:0] ALUCtrl;
            reg [4:0] pass1, pass2;
            reg regDstPass=0;
            wire [31:0] jumpAddress; //not for JR
            
            assign opcode = instruction_reg_IF[31:26];
            assign funct = instruction_reg_IF[5:0];
    
            
            //parameters for the ALU ctrl output based on its proper function
            localparam addALU = 4'b0010;
            localparam subALU = 4'b0110;
            localparam andALU = 4'b0000;
            localparam orALU = 4'b0001;
            localparam sltALU = 4'b0111;
            localparam norALU = 4'b1100;
            
            //parameters for opcodes
            localparam lwOP = 6'b100011;
            localparam swOP = 6'b101011;
            localparam beqOP = 6'b000100;
            localparam bneOP = 6'b000101;
            localparam addiOP = 6'b001000;
            localparam andiOP = 6'b001100;
            localparam oriOP = 6'b001101;
            localparam jumpOP = 6'b000010;
            localparam jalOP = 6'b000011;
            localparam ROP = 6'b000000;
            
            //parameters funct fields of arithmetic operations
            localparam addFUNCT = 6'b100000;
            localparam subFUNCT = 6'b100010;
            localparam andFUNCT = 6'b100100;
            localparam orFUNCT = 6'b100101;
            localparam sltFUNCT = 6'b101010;
            localparam norFUNCT = 6'b100111;
            localparam jrFUNCT = 6'b001000;
            
            assign ALUCtrl = (opcode == lwOP)? addALU:
                             (opcode == swOP)? addALU:
                             (opcode == beqOP)? subALU:
                             (opcode == bneOP) ? subALU:
                             (opcode == addiOP)? addALU:
                             (opcode == andiOP)? andALU:
                             (opcode == oriOP)? orALU:
                             arithmetic; //if it is jump the value doesn't matter, if not then it will send through arithmetic instruction
            
            //assigning ALUCtrl if the opcode == 000000, meaning it is an arithmetic instruction
            assign arithmetic = (funct == addFUNCT)? addALU:
                                (funct ==  subFUNCT)? subALU:
                                (funct == andFUNCT)? andALU:
                                (funct == orFUNCT)? orALU:
                                (funct == norFUNCT)? norALU:
                                sltALU;
                                
            //end of ALU assignment/////////////////////////////////////////////////////////////////////////////////////
            
            //control for PC////////////////////////////////////////////////////////////////////////////////////////////
             //if the opcode says to jump, assign to a 1
//            assign PCSrc = (opcode == beqOP) && (Zero_EX)? 1: 0; //if zero is high and the opcode is beq, the branch will be taken
            //end of control for PC/////////////////////////////////////////////////////////////////////////////////////
            
            
            //rest of datapath control//////////////////////////////////////////////////////////////////////////////////
            assign regDst = (opcode == ROP)? 1: 0;
            assign ALUSrc = (opcode == lwOP) || (opcode == oriOP) || (opcode == andiOP) || (opcode == addiOP) || (opcode == swOP)? 1: 0;
            assign MemtoReg = (opcode == lwOP)? 1: 0; //only asserted for load word
            assign RegWrite = (opcode == lwOP) || (opcode == oriOP) || (opcode == andiOP) || (opcode == addiOP) || (opcode == ROP)? 1: 0; 
            assign MemRead = (opcode == lwOP)? 1: 0;
            assign MemWrite = (opcode == swOP)? 1: 0;
            assign branch = (opcode == beqOP)? 1: 0;
            assign branchBNE = (opcode == bneOP)?1:0;
            assign ZeroExt = (opcode == oriOP) || (opcode == andiOP)? 1: 0; //only zero extend for an andi or ori op     
            //end of datapath control///////////////////////////////////////////////////////////////////////////////////
            
            
           assign jumpAddress = {pc_reg_IF[31:28], instruction_reg_IF[25:0], 2'b00};
           assign jump = (opcode == jumpOP) && (jump_ID == 2'b00) && (jump_EX == 2'b00) && (!PCSrc_WB) ? 2'b01: //it will only assert jump if there are no other jumps in the pipeline
                         (opcode == jalOP) && (jump_ID == 2'b00) && (jump_EX == 2'b00) && (!PCSrc_WB) ? 2'b10:
                         (funct == jrFUNCT) && (jump_ID == 2'b00) && (jump_EX == 2'b00) &&  (!PCSrc_WB)? 2'b11: 2'b00;                      
                                    
                       //jump = 1 rgular jump, jump = 2 jump and link, jump = 3 jr, else we ain't jumpin
            reg thisIsNoop_ID, thisIsNoop_EX, thisIsNoop_MEM, stall_Noop;
            //assign to registers for next stages///////////////////////////////////////////
            always@(posedge clk) begin
                if(insert_ex_noop == 0) begin
                    ALUSrc_ID <= ALUSrc;
                    jumpAddress_ID <= jumpAddress;
                    jump_ID <= jump;
                    Branch_ID <= branch;
                    BranchBNE_ID <= branchBNE;
                    MemWrite_ID <= MemWrite;
                    MemRead_ID <= MemRead;
                    RegWrite_ID <= RegWrite;
                    MemtoReg_ID <= MemtoReg;
                    ALUCtrl_ID <= ALUCtrl;
                    pc_reg_ID <= pc_reg_IF; 
                    extend_ID <= extend;
                    thisIsNoop_ID <= 0;
                    instructionID <= instruction_reg_IF;
                   // stall_Noop <= 0;
    //                WriteReg_ID <= WriteReg; //<= (regDst == 1) ? instruction_reg_IF [15:11]: //if RegDst == 1 then 15 - 11, otherwise 20 - 16
    //                                                  instruction_reg_IF [20:16];
                    //Data1 and Data2 are already own registers so they can just go across, delay them once through a register
//                    Data1_ID <= Data1; 
//                    Data2_ID <= Data2;
                    pass1 <= instruction_reg_IF [15:11];
                    pass2 <= instruction_reg_IF [20:16];
                    regDstPass <= regDst;
                    ID_Rs <= instruction_reg_IF [25:21];
                    ID_Rt <= instruction_reg_IF [20:16];
                    ID_Rd <= (opcode == lwOP) || (opcode == oriOP) || (opcode == andiOP) || (opcode == addiOP) ? instruction_reg_IF [20:16] : instruction_reg_IF [15:11]; //compensates for instructions with destination register in rt field
                end
                else begin //make sure all the registers are present here                    
                    ALUSrc_ID <= 0;
                    Branch_ID <= 0;
                    BranchBNE_ID <= 0;
                    MemWrite_ID <= 0;
                    MemRead_ID <= 0;
                    RegWrite_ID <= 0;
                    MemtoReg_ID <= 0;
                    instructionID <= 0;
                    ALUCtrl_ID <= 0; 
                    jumpAddress_ID <= 0;
                    jump_ID <= 0;
//                    Data1 <= 0;
//                    Data2 <= 0;
                    pc_reg_ID <= 0;
                    extend_ID <= 0;
                    WriteReg_ID <= 0;
//                    Data1_ID <= 0;
//                    Data2_ID <= 0;
                    pass1 <= 0;
                    pass2 <= 0;
                    ID_Rs <= 0;
                    ID_Rt <= 0;
                    ID_Rd <= 0;
                    thisIsNoop_ID <= 1;
                    //stall_Noop <= 1;
                 end
            end
            //end of assignment/////////////////////////////////////////////////////////////
    
            //initialize to zero
            initial begin
                ALUSrc_ID <= 0;
                Branch_ID <= 0;
                BranchBNE_ID <= 0;
                MemWrite_ID <= 0;
                MemRead_ID <= 0;
                RegWrite_ID <= 0;
                instructionID <= 0;
                MemtoReg_ID <= 0;
                ALUCtrl_ID <= 0; 
                Data1 <= 0;
                Data2 <= 0;                
                pc_reg_ID <= 0;
                extend_ID <= 0;
                WriteReg_ID <= 0;
//                Data1_ID <= 0;
//                Data2_ID <= 0;
                pass1 <= 0;
                pass2 <= 0;
                ID_Rs <= 0;
                ID_Rt <= 0;
                ID_Rd <= 0;
                jumpAddress_ID <= 0;
                jump_ID <= 0;
                thisIsNoop_ID <= 0;
                stall_Noop <= 0;
                //regDstPass <=0;
            end
    //End of ID////////////////////////////////////////////////////////////////////////
    
    //EX/////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //EX Stage
        
        reg [1:0] ALU_Zero;
        reg [5:0] WriteReg_EX; 
        reg [31:0] Data2_EX;
        reg Branch_EX, BranchBNE_EX, MemWrite_EX, MemRead_EX, MemtoReg_EX, RegWrite_EX;
        
    //ALU creation////////////////////////////////////////////////////////////// 
        //inputs to ALU (ALUCtrl is part of top-level)
        wire [31:0] A, B;
        wire Zero;
        
        //outputs from the ALU (zero is part of top-level)
        reg [31:0] ALUOut, ALU_EX;  
        
        //assignments
        assign Zero = (ALUOut == 0) ? 1: 0; //zero becomes one if the output of the ALU is 0
        assign A = (forwardA == 2'b10) ? ALU_EX:
                   (forwardA == 2'b01) ? WriteData:
                   Data1; 
        assign B = (forwardB == 2'b11) ? extend_ID:
                   (forwardB == 2'b10) ? ALU_EX:
                   (forwardB == 2'b01) ? WriteData:
                   Data2;
        
        always @(ALUCtrl_ID, A, B) begin
            case(ALUCtrl_ID)
                0: ALUOut <= A & B;
                1: ALUOut <= A | B;
                2: ALUOut <= A + B;
                6: ALUOut <= A - B;
                7: ALUOut <= $signed(A) < $signed(B) ? 1: 0; //sets to 1 is A < B
                12: ALUOut <= ~(A | B); //nor
                default: ALUOut <= 0;
            endcase
        end  
        //end of ALU///////////////////////////////////////////////////////////////
        
        assign WriteReg = regDstPass ? pass1 : pass2;        
        
        //compute branch target address
        wire [32:0] signExtendPC;  
        assign signExtendPC = extend_ID << 2;
        always@(posedge clk)
             branch_target_EX <= pc_reg_ID + signExtendPC;
        //done computing branch target address
        
        reg [1:0] jump_EX, jump_MEM;
        
        always@(posedge clk) begin
            ALU_Zero <= Zero; //this may be happening a clock cycle too late
            WriteReg_EX <= WriteReg; //register write address
            //ALUOut is already a register
            ALU_EX <= ALUOut;
            //branch_target_EX is the branch target address
            Branch_EX <= Branch_ID;
            BranchBNE_EX <= BranchBNE_ID;
            MemWrite_EX <= MemWrite_ID; 
            MemRead_EX <= MemRead_ID; 
            MemtoReg_EX <= MemtoReg_ID; 
            RegWrite_EX <= RegWrite_ID;
            Data2_EX <= Data2; //switched this from B
            EX_Rd <= ID_Rd;
            EX_Rs <= ID_Rs; 
            instructionEX <= instructionID;
            EX_Rt <= ID_Rt;
            jump_EX <= jump_ID;
            thisIsNoop_EX <= thisIsNoop_ID;
        end   
        
          //initialize to zero
       initial begin
           ALU_Zero <= 0;
           WriteReg_EX <= 0;
           Branch_EX <= 0;
           BranchBNE_EX <= 0;
           instructionEX <= 0;
           MemWrite_EX <= 0;
           MemRead_EX <= 0;
           MemtoReg_EX <= 0;
           RegWrite_EX <= 0;
           Data2_EX <= 0; 
           branch_target_EX <= 0;
           ALU_EX <= 0;
           EX_Rd <= 0;
           EX_Rs <= 0; 
           EX_Rt <= 0;
           jump_EX <= 0;
           thisIsNoop_EX <= 0;
       end    
    //End of EX//////////////////////////////////////////////////////////////////////////////////////////////
    
    //MEM////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //need a wire to replace Data2_EX, value will be ALU_EX if not forwarded, but will be WB value if forwarded
    wire [31:0] mem_storage;    
    assign mem_storage = (forwardC == 1) ? WriteData:
                         Data2_EX; //This is the output of input B from the last stage
    // Synchronous read or write
      always@(posedge clk)
        // decode address to start at location 0x1000
        if (ALU_EX >= 32'h00001000 && ALU_EX <= 32'h00001fff) begin
          if (MemRead_EX)
            data_output_MEM <= d_mem[(ALU_EX-32'h1000)>>2];
          if (MemWrite_EX)
            d_mem[(ALU_EX-32'h1000)>>2] <= mem_storage;
        end
       
       assign PCSrc_MEM = (ALU_Zero && Branch_EX) ?  1: 
                (!ALU_Zero) & (BranchBNE_EX) ? 1 : 0;
         
       always@(posedge clk) begin
            MemtoReg_MEM <= MemtoReg_EX;
            RegWrite_MEM <= RegWrite_EX;
            ALUOut_MEM <= ALU_EX;
            WriteReg_MEM <= WriteReg_EX;
            MEM_Rd <= EX_Rd;
            MEM_Rs <= EX_Rs; 
            instructionMEM <= instructionEX;
            MEM_Rt <= EX_Rt;
            thisIsNoop_MEM <= thisIsNoop_EX;
            jump_MEM <= jump_EX;
        end
        
        initial begin
            data_output_MEM <= 0;
            MemtoReg_MEM <= 0;
            RegWrite_MEM <= 0;
            ALUOut_MEM <= 0;
            instructionMEM <= 0;
            WriteReg_MEM <= 0;
            MEM_Rd <= 0;
            MEM_Rs <= 0; 
            MEM_Rt <= 0;
            thisIsNoop_MEM <= 0;
            jump_MEM <= 0;
        end
     //End of MEM/////////////////////////////////////////////////////////////////////////////////////////////
     
     //WB/////////////////////////////////////////////////////////////////////////////////////////////////////
      assign WriteData = (MemtoReg_MEM == 1) ? data_output_MEM: ALUOut_MEM;
     //End of WB//////////////////////////////////////////////////////////////////////////////////////////////
     
     //assign top-level ports here
         assign PC = pc_reg;    
         assign Instruction = instruction_reg_IF; //instruction going into the ID stage
         assign ALUResult = ALUOut;
         assign dWriteData = mem_storage; //multiplexed data going into data memory
         assign WriteBackData = WriteData;
         
    //forwarding logic
//    assign forwardA = ((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd == ID_Rs) && (jump == 2'b00)) ? 2'b10: // && !((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd != ID_Rs))
//                      ((RegWrite_MEM == 1) && (MEM_Rd != 0) && (MEM_Rd == ID_Rs) && (jump == 2'b00)) ? 2'b01:
//                      2'b00;
                      
//    assign forwardB = (ALUSrc_ID == 1'b1)? 2'b11:
//                      ((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd == ID_Rt) && (jump == 2'b00)) ? 2'b10: // && !((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd != ID_Rt))
//                      ((RegWrite_MEM == 1) && (MEM_Rd != 0) && (MEM_Rd == ID_Rt) && (jump == 2'b00)) ? 2'b01:
//                      2'b00;
                      
    assign forwardA = ((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd == ID_Rs) /*&& (!thisIsNoop_EX)*/) ? 2'b10: // && !((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd != ID_Rs))
                      ((RegWrite_MEM == 1) && (MEM_Rd != 0) && (MEM_Rd == ID_Rs) && (!thisIsNoop_MEM)) ? 2'b01:
                      2'b00;
                      
    assign forwardB = (ALUSrc_ID == 1'b1)? 2'b11:
                      ((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd == ID_Rt) && (!thisIsNoop_EX)) ? 2'b10: // && !((RegWrite_EX == 1) && (EX_Rd != 0) && (EX_Rd != ID_Rt))
                      ((RegWrite_MEM == 1) && (MEM_Rd != 0) && (MEM_Rd == ID_Rt) && (!thisIsNoop_MEM)) ? 2'b01:
                      2'b00;
                      
    //shouldn't need to check for a store word instruction b/c if it isn't sw then what goes into memory doesn't matter (memwrite is off)
    assign forwardC = ((RegWrite_MEM == 1) && (MEM_Rd != 0) && (MEM_Rd == EX_Rt))? 1: 0; //high if we should forward
    
    //stall logic///////////////////////////////////////////////////////////////////////////////////////////
    wire stallPipe;
    reg PCSrc_WB;
    initial
        PCSrc_WB <= 0;
    
    always @ (posedge clk) begin
        PCSrc_WB <= PCSrc_MEM;
    end
    
    
    //stalling for lw
    assign stallPipe = (MemRead_ID && ((ID_Rt == instruction_reg_IF[25:21]) || (ID_Rt == instruction_reg_IF[20:16]))) ? 1: 0;
    
    //branch stalling
    assign halt_pc = (jump != 2'b00) ? 1: //stalls for jump in ID stage
                     (stallPipe) ? 1: //stall for lw
                     (Branch_ID) ? 1: //stall PC for branch going into EX
                     (BranchBNE_ID) ? 1 :
                     (opcode == beqOP && Branch_ID == 0 && Branch_EX == 0) ? 1: 
                     (opcode == bneOP && BranchBNE_ID == 0 && BranchBNE_EX == 0) ? 1: 0; //stall PC if ID is a branch
                     
    assign insert_ex_noop = (jump_ID != 0)? 1: //logic to insert two noops for jumps
                            (jump_EX != 0)? 1:
                            (stallPipe) ? 1: //stall for lw
                            (Branch_ID) ? 1: //branch in EX stage
                            (BranchBNE_ID) ? 1:
                            (BranchBNE_EX) ? 1:
                            (Branch_EX) ? 1: //branch in MEM stage
                            (PCSrc_WB) ? 1: 0; //there is a branch in the WB stage and it is taken
    
    //end of stall logic////////////////////////////////////////////////////////////////////////////////////
     
endmodule



