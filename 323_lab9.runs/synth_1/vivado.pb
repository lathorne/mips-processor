
x
Command: %s
53*	vivadotcl2G
3synth_design -top mips_final -part xc7a100tcsg324-12default:defaultZ4-113h px� 
:
Starting synth_design
149*	vivadotclZ4-321h px� 
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xc7a100t2default:defaultZ17-347h px� 
�
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xc7a100t2default:defaultZ17-349h px� 
�
&Analyzing %s file "%s" into library %s1936*verific2
Verilog2default:default2R
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
work2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
12default:default8@Z9-2216h px� 
�
=concatenation with unsized literal; will interpret as 32 bits1849*oasys2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
1632default:default8@Z8-1849h px� 
�
%s*synth2�
sStarting Synthesize : Time (s): cpu = 00:00:05 ; elapsed = 00:00:12 . Memory (MB): peak = 321.645 ; gain = 109.406
2default:defaulth px� 
�
synthesizing module '%s'638*oasys2

mips_final2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
232default:default8@Z8-638h px� 
Q
%s
*synth29
%	Parameter addALU bound to: 4'b0010 
2default:defaulth p
x
� 
Q
%s
*synth29
%	Parameter subALU bound to: 4'b0110 
2default:defaulth p
x
� 
Q
%s
*synth29
%	Parameter andALU bound to: 4'b0000 
2default:defaulth p
x
� 
P
%s
*synth28
$	Parameter orALU bound to: 4'b0001 
2default:defaulth p
x
� 
Q
%s
*synth29
%	Parameter sltALU bound to: 4'b0111 
2default:defaulth p
x
� 
Q
%s
*synth29
%	Parameter norALU bound to: 4'b1100 
2default:defaulth p
x
� 
Q
%s
*synth29
%	Parameter lwOP bound to: 6'b100011 
2default:defaulth p
x
� 
Q
%s
*synth29
%	Parameter swOP bound to: 6'b101011 
2default:defaulth p
x
� 
R
%s
*synth2:
&	Parameter beqOP bound to: 6'b000100 
2default:defaulth p
x
� 
R
%s
*synth2:
&	Parameter bneOP bound to: 6'b000101 
2default:defaulth p
x
� 
S
%s
*synth2;
'	Parameter addiOP bound to: 6'b001000 
2default:defaulth p
x
� 
S
%s
*synth2;
'	Parameter andiOP bound to: 6'b001100 
2default:defaulth p
x
� 
R
%s
*synth2:
&	Parameter oriOP bound to: 6'b001101 
2default:defaulth p
x
� 
S
%s
*synth2;
'	Parameter jumpOP bound to: 6'b000010 
2default:defaulth p
x
� 
R
%s
*synth2:
&	Parameter jalOP bound to: 6'b000011 
2default:defaulth p
x
� 
P
%s
*synth28
$	Parameter ROP bound to: 6'b000000 
2default:defaulth p
x
� 
U
%s
*synth2=
)	Parameter addFUNCT bound to: 6'b100000 
2default:defaulth p
x
� 
U
%s
*synth2=
)	Parameter subFUNCT bound to: 6'b100010 
2default:defaulth p
x
� 
U
%s
*synth2=
)	Parameter andFUNCT bound to: 6'b100100 
2default:defaulth p
x
� 
T
%s
*synth2<
(	Parameter orFUNCT bound to: 6'b100101 
2default:defaulth p
x
� 
U
%s
*synth2=
)	Parameter sltFUNCT bound to: 6'b101010 
2default:defaulth p
x
� 
U
%s
*synth2=
)	Parameter norFUNCT bound to: 6'b100111 
2default:defaulth p
x
� 
T
%s
*synth2<
(	Parameter jrFUNCT bound to: 6'b001000 
2default:defaulth p
x
� 
�
,$readmem data file '%s' is read successfully3426*oasys20
final_instruction_memory.txt2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
942default:default8@Z8-3876h px� 
�
,$readmem data file '%s' is read successfully3426*oasys2)
final_data_memory.txt2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
952default:default8@Z8-3876h px� 
�
merging register '%s' into '%s'3619*oasys2#
MemtoReg_ID_reg2default:default2"
MemRead_ID_reg2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
2642default:default8@Z8-4471h px� 
�
merging register '%s' into '%s'3619*oasys2"
ID_Rt_reg[4:0]2default:default2"
pass2_reg[4:0]2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
2802default:default8@Z8-4471h px� 
�
merging register '%s' into '%s'3619*oasys2#
MemtoReg_EX_reg2default:default2"
MemRead_EX_reg2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
4052default:default8@Z8-4471h px� 
�
%done synthesizing module '%s' (%s#%s)256*oasys2

mips_final2default:default2
12default:default2
12default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
232default:default8@Z8-256h px� 
�
%s*synth2�
sFinished Synthesize : Time (s): cpu = 00:00:06 ; elapsed = 00:00:14 . Memory (MB): peak = 344.395 ; gain = 132.156
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
~Finished Constraint Validation : Time (s): cpu = 00:00:06 ; elapsed = 00:00:15 . Memory (MB): peak = 344.395 ; gain = 132.156
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Loading part: xc7a100tcsg324-1
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Loading Part and Timing Information : Time (s): cpu = 00:00:06 ; elapsed = 00:00:15 . Memory (MB): peak = 344.395 ; gain = 132.156
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
W
Loading part %s157*device2$
xc7a100tcsg324-12default:defaultZ21-403h px� 
�
}HDL ADVISOR - The operator resource <%s> is shared. To prevent sharing consider applying a KEEP on the output of the operator4233*oasys2
adder2default:default2T
>J:/Documents/323_lab9/323_lab9.srcs/sources_1/new/mips_final.v2default:default2
3622default:default8@Z8-5818h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[31]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[30]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[29]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[28]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[27]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[26]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[25]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[24]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[23]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[22]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[21]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[20]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[19]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[18]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[17]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[16]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[15]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[14]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[13]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[12]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[11]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[10]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[9]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[8]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[7]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[6]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[5]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[4]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[3]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[2]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[1]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[0]2default:defaultZ8-5546h px� 
v
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
branch2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	branchBNE2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	MemWrite02default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	MemtoReg02default:defaultZ8-5546h px� 
{
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
arithmetic32default:defaultZ8-5546h px� 
w
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
regDst02default:defaultZ8-5546h px� 
�
^ROM "%s" won't be mapped to RAM because address size (%s) is larger than maximum supported(%s)3997*oasys2
Zero02default:default2
322default:default2
252default:defaultZ8-5545h px� 
u
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
i_mem2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[31]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[30]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[29]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[28]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[27]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[26]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[25]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[24]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[23]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[22]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[21]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[20]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[19]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[18]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[17]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[16]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[15]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[14]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[13]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[12]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[11]2default:defaultZ8-5546h px� 
z
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2

RF_reg[10]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[9]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[8]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[7]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[6]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[5]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[4]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[3]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[2]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[1]2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	RF_reg[0]2default:defaultZ8-5546h px� 
v
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
branch2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	branchBNE2default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	MemWrite02default:defaultZ8-5546h px� 
y
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
	MemtoReg02default:defaultZ8-5546h px� 
{
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
arithmetic32default:defaultZ8-5546h px� 
w
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
regDst02default:defaultZ8-5546h px� 
�
^ROM "%s" won't be mapped to RAM because address size (%s) is larger than maximum supported(%s)3997*oasys2
Zero02default:default2
322default:default2
252default:defaultZ8-5545h px� 
u
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
i_mem2default:defaultZ8-5546h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
	forwardA02default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
	forwardB12default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
ALUOut02default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
forwardC2default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2

PCSrc_MEM02default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
	stallPipe2default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
ALUSrc02default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2"
jumpAddress_ID2default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
	RegWrite02default:default2
12default:default2
52default:defaultZ8-5544h px� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
ZeroExt2default:default2
12default:default2
52default:defaultZ8-5544h px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:07 ; elapsed = 00:00:15 . Memory (MB): peak = 390.594 ; gain = 178.355
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
� 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input     32 Bit       Adders := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   3 Input     32 Bit       Adders := 1     
2default:defaulth p
x
� 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	               32 Bit    Registers := 45    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                5 Bit    Registers := 9     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                4 Bit    Registers := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                2 Bit    Registers := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                1 Bit    Registers := 18    
2default:defaulth p
x
� 
8
%s
*synth2 
+---RAMs : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	              32K Bit         RAMs := 1     
2default:defaulth p
x
� 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	  48 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input     32 Bit        Muxes := 11    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   4 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   3 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   7 Input     32 Bit        Muxes := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   9 Input      9 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   7 Input      7 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      5 Bit        Muxes := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   5 Input      5 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   6 Input      5 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      4 Bit        Muxes := 4     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   5 Input      4 Bit        Muxes := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   8 Input      3 Bit        Muxes := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      3 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      2 Bit        Muxes := 22    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 36    
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
Y
%s
*synth2A
-Start RTL Hierarchical Component Statistics 
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
O
%s
*synth27
#Hierarchical RTL Component report 
2default:defaulth p
x
� 
?
%s
*synth2'
Module mips_final 
2default:defaulth p
x
� 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
� 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input     32 Bit       Adders := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   3 Input     32 Bit       Adders := 1     
2default:defaulth p
x
� 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	               32 Bit    Registers := 45    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                5 Bit    Registers := 9     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                4 Bit    Registers := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                2 Bit    Registers := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                1 Bit    Registers := 18    
2default:defaulth p
x
� 
8
%s
*synth2 
+---RAMs : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	              32K Bit         RAMs := 1     
2default:defaulth p
x
� 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	  48 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input     32 Bit        Muxes := 11    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   4 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   3 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   7 Input     32 Bit        Muxes := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   9 Input      9 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   7 Input      7 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      5 Bit        Muxes := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   5 Input      5 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   6 Input      5 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      4 Bit        Muxes := 4     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   5 Input      4 Bit        Muxes := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   8 Input      3 Bit        Muxes := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      3 Bit        Muxes := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      2 Bit        Muxes := 22    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 36    
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
[
%s
*synth2C
/Finished RTL Hierarchical Component Statistics
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s
*synth2k
WPart Resources:
DSPs: 240 (col length:80)
BRAMs: 270 (col length: RAMB18 80 RAMB36 40)
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
[ROM "%s" won't be mapped to Block RAM because address size (%s) smaller than threshold (%s)3996*oasys2
p_0_out02default:default2
22default:default2
52default:defaultZ8-5544h px� 
�
^ROM "%s" won't be mapped to RAM because address size (%s) is larger than maximum supported(%s)3997*oasys2
Zero02default:default2
322default:default2
252default:defaultZ8-5545h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2 
pass1_reg[3]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[16]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2 
pass1_reg[4]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[17]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2 
pass1_reg[0]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[13]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2 
pass1_reg[1]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[14]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2 
pass1_reg[2]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[15]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
pc_reg_ID_reg[28]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[28]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
pc_reg_ID_reg[29]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[29]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
pc_reg_ID_reg[30]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[30]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
pc_reg_ID_reg[31]2default:default2
FDR2default:default2*
jumpAddress_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[0]2default:default2
FDR2default:default2)
jumpAddress_ID_reg[1]2default:defaultZ8-3886h px� 
�
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2+
\jumpAddress_ID_reg[1] 2default:defaultZ8-3333h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[2]2default:default2
FDR2default:default2$
extend_ID_reg[0]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[3]2default:default2
FDR2default:default2$
extend_ID_reg[1]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[4]2default:default2
FDR2default:default2$
extend_ID_reg[2]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[5]2default:default2
FDR2default:default2$
extend_ID_reg[3]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[6]2default:default2
FDR2default:default2$
extend_ID_reg[4]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[7]2default:default2
FDR2default:default2$
extend_ID_reg[5]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[8]2default:default2
FDR2default:default2$
extend_ID_reg[6]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2)
jumpAddress_ID_reg[9]2default:default2
FDR2default:default2$
extend_ID_reg[7]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[10]2default:default2
FDR2default:default2$
extend_ID_reg[8]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[11]2default:default2
FDR2default:default2$
extend_ID_reg[9]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[12]2default:default2
FDR2default:default2%
extend_ID_reg[10]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[13]2default:default2
FDR2default:default2%
extend_ID_reg[11]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[14]2default:default2
FDR2default:default2%
extend_ID_reg[12]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[15]2default:default2
FDR2default:default2%
extend_ID_reg[13]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[16]2default:default2
FDR2default:default2%
extend_ID_reg[14]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[17]2default:default2
FDR2default:default2%
extend_ID_reg[15]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[18]2default:default2
FDR2default:default2 
pass2_reg[0]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[19]2default:default2
FDR2default:default2 
pass2_reg[1]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[20]2default:default2
FDR2default:default2 
pass2_reg[2]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[21]2default:default2
FDR2default:default2 
pass2_reg[3]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[22]2default:default2
FDR2default:default2 
pass2_reg[4]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[23]2default:default2
FDR2default:default2 
ID_Rs_reg[0]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[24]2default:default2
FDR2default:default2 
ID_Rs_reg[1]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[25]2default:default2
FDR2default:default2 
ID_Rs_reg[2]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[26]2default:default2
FDR2default:default2 
ID_Rs_reg[3]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2*
jumpAddress_ID_reg[27]2default:default2
FDR2default:default2 
ID_Rs_reg[4]2default:defaultZ8-3886h px� 
�
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2%
\ALU_Zero_reg[1] 2default:defaultZ8-3333h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[16]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[17]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[18]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[19]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[20]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[21]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[22]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[23]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[24]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[25]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[26]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[27]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[28]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[29]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2%
extend_ID_reg[30]2default:default2
FDR2default:default2%
extend_ID_reg[31]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2-
instruction_reg_IF_reg[6]2default:default2
FDE2default:default2-
instruction_reg_IF_reg[7]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2-
instruction_reg_IF_reg[8]2default:default2
FDE2default:default2-
instruction_reg_IF_reg[9]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2-
instruction_reg_IF_reg[9]2default:default2
FDE2default:default2.
instruction_reg_IF_reg[10]2default:defaultZ8-3886h px� 
�
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default20
\instruction_reg_IF_reg[20] 2default:defaultZ8-3333h px� 
�
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2"
\EX_Rt_reg[4] 2default:defaultZ8-3333h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2$
extend_ID_reg[6]2default:default2
FDR2default:default2$
extend_ID_reg[7]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2$
extend_ID_reg[8]2default:default2
FDR2default:default2%
extend_ID_reg[10]2default:defaultZ8-3886h px� 
�
"merging instance '%s' (%s) to '%s'3436*oasys2$
extend_ID_reg[9]2default:default2
FDR2default:default2%
extend_ID_reg[10]2default:defaultZ8-3886h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2.
instruction_reg_IF_reg[20]2default:default2

mips_final2default:defaultZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2)
jumpAddress_ID_reg[1]2default:default2

mips_final2default:defaultZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2 
EX_Rt_reg[4]2default:default2

mips_final2default:defaultZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2#
ALU_Zero_reg[1]2default:default2

mips_final2default:defaultZ8-3332h px� 
�
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2 
pass2_reg[4]2default:default2

mips_final2default:defaultZ8-3332h px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:14 ; elapsed = 00:00:24 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Start ROM, RAM, DSP and Shift Register Reporting
2default:defaulth px� 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px� 
e
%s*synth2M
9
Block RAM: Preliminary Mapping  Report (see note below)
2default:defaulth px� 
�
%s*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth px� 
�
%s*synth2�
�|Module Name | RTL Object | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
2default:defaulth px� 
�
%s*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth px� 
�
%s*synth2�
�|mips_final  | d_mem_reg  | 1 K x 32(READ_FIRST)   | W |   | 1 K x 32(WRITE_FIRST)  |   | R | Port A and B     | 0      | 1      | 
2default:defaulth px� 
�
%s*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

2default:defaulth px� 
�
%s*synth2�
�Note: The table above is a preliminary report that shows the Block RAMs at the current stage of the synthesis flow. Some Block RAMs may be reimplemented as non Block RAM primitives later in the synthesis flow. Multiple instantiated Block RAMs are reported only once. 
2default:defaulth px� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Finished ROM, RAM, DSP and Shift Register Reporting
2default:defaulth px� 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px� 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
F
%s
*synth2.
Start Timing Optimization
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
|Finished Timing Optimization : Time (s): cpu = 00:00:14 ; elapsed = 00:00:24 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
�The timing for the instance %s (implemented as a block RAM) might be sub-optimal as no optional output register could be merged into the block ram. Providing additional output register may help in improving timing.
3630*oasys2
	d_mem_reg2default:defaultZ8-4480h px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
{Finished Technology Mapping : Time (s): cpu = 00:00:14 ; elapsed = 00:00:25 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
Q
%s
*synth29
%Start Flattening Before IO Insertion
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
T
%s
*synth2<
(Finished Flattening Before IO Insertion
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
uFinished IO Insertion : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
D
%s
*synth2,

Report Check Netlist: 
2default:defaulth p
x
� 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
� 
u
%s
*synth2]
I|      |Item              |Errors |Warnings |Status |Description       |
2default:defaulth p
x
� 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
� 
u
%s
*synth2]
I|1     |multi_driven_nets |      0|        0|Passed |Multi driven nets |
2default:defaulth p
x
� 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Renaming Generated Instances : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
L
%s
*synth24
 Start Rebuilding User Hierarchy
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Start Renaming Generated Ports
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Renaming Generated Ports : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
J
%s
*synth22
Start Renaming Generated Nets
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Renaming Generated Nets : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
� 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
� 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
� 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
� 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
� 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px� 
F
%s*synth2.
+------+---------+------+
2default:defaulth px� 
F
%s*synth2.
|      |Cell     |Count |
2default:defaulth px� 
F
%s*synth2.
+------+---------+------+
2default:defaulth px� 
F
%s*synth2.
|1     |BUFG     |     1|
2default:defaulth px� 
F
%s*synth2.
|2     |CARRY4   |    28|
2default:defaulth px� 
F
%s*synth2.
|3     |LUT1     |    34|
2default:defaulth px� 
F
%s*synth2.
|4     |LUT2     |    71|
2default:defaulth px� 
F
%s*synth2.
|5     |LUT3     |    77|
2default:defaulth px� 
F
%s*synth2.
|6     |LUT4     |    44|
2default:defaulth px� 
F
%s*synth2.
|7     |LUT5     |   318|
2default:defaulth px� 
F
%s*synth2.
|8     |LUT6     |   564|
2default:defaulth px� 
F
%s*synth2.
|9     |MUXF7    |   193|
2default:defaulth px� 
F
%s*synth2.
|10    |MUXF8    |    64|
2default:defaulth px� 
F
%s*synth2.
|11    |RAMB36E1 |     1|
2default:defaulth px� 
F
%s*synth2.
|12    |FDRE     |  1411|
2default:defaulth px� 
F
%s*synth2.
|13    |FDSE     |    16|
2default:defaulth px� 
F
%s*synth2.
|14    |IBUF     |     1|
2default:defaulth px� 
F
%s*synth2.
|15    |OBUF     |   160|
2default:defaulth px� 
F
%s*synth2.
+------+---------+------+
2default:defaulth px� 
E
%s
*synth2-

Report Instance Areas: 
2default:defaulth p
x
� 
N
%s
*synth26
"+------+---------+-------+------+
2default:defaulth p
x
� 
N
%s
*synth26
"|      |Instance |Module |Cells |
2default:defaulth p
x
� 
N
%s
*synth26
"+------+---------+-------+------+
2default:defaulth p
x
� 
N
%s
*synth26
"|1     |top      |       |  2983|
2default:defaulth p
x
� 
N
%s
*synth26
"+------+---------+-------+------+
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Writing Synthesis Report : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
r
%s
*synth2Z
FSynthesis finished with 0 errors, 0 critical warnings and 6 warnings.
2default:defaulth p
x
� 
�
%s
*synth2�
~Synthesis Optimization Runtime : Time (s): cpu = 00:00:13 ; elapsed = 00:00:17 . Memory (MB): peak = 629.613 ; gain = 394.219
2default:defaulth p
x
� 
�
%s
*synth2�
Synthesis Optimization Complete : Time (s): cpu = 00:00:15 ; elapsed = 00:00:26 . Memory (MB): peak = 629.613 ; gain = 417.375
2default:defaulth p
x
� 
B
 Translating synthesized netlist
350*projectZ1-571h px� 
f
-Analyzing %s Unisim elements for replacement
17*netlist2
302default:defaultZ29-17h px� 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px� 
�
�Netlist '%s' is not ideal for floorplanning, since the cellview '%s' contains a large number of primitives.  Please consider enabling hierarchy in synthesis if you want to do floorplanning.
310*netlist2

mips_final2default:default2

mips_final2default:defaultZ29-101h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px� 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px� 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1722default:default2
72default:default2
02default:default2
02default:defaultZ4-41h px� 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
synth_design: 2default:default2
00:00:162default:default2
00:00:262default:default2
629.6132default:default2
403.3202default:defaultZ17-268h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2N
:J:/Documents/323_lab9/323_lab9.runs/synth_1/mips_final.dcp2default:defaultZ17-1381h px� 
�
sreport_utilization: Time (s): cpu = 00:00:00 ; elapsed = 00:00:00.074 . Memory (MB): peak = 629.613 ; gain = 0.000
*commonh px� 
�
Exiting %s at %s...
206*common2
Vivado2default:default2,
Tue Apr 03 14:12:39 20182default:defaultZ17-206h px� 


End Record