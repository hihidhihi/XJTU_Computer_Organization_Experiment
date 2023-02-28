**实验六 CPU综合设计**

**一、实验目的**

1.  掌握复杂系统设计方法。

2.  深刻理解计算机系统硬件原理。

**二、实验内容**

1.  设计一个基于MIPS指令集的CPU，支持以下指令：{addu, subu, ori, lw, sw,
    beq, lui, nop}（及格）

2.  CPU需要包含寄存器组、RAM模块、ALU模块、指令译码模块。

3.  该CPU能运行基本的汇编指令（编写测试程序完成所有指令测试，要求与MARS模拟器运行结果一致）。

4.  在1基础上，扩展指令集，实现MIPS-Lite指令，见下页。（A-\~A，编写测试程序完成所有指令测试，要求与MARS模拟器运行结果一致）

5.  在4基础上，实现5级流水线CPU。（A+，编写测试程序完成所有指令测试，要求与MARS模拟器运行结果一致）

6.  如发现代码为网上下载代码，成绩一律按不及格处理。

**三、实验要求**

1.  编写相应测试程序，完成所有指令测试。

**四、实验代码及结果**

五级流水线数据通路：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image1.png)

设计思路：将CPU工作分为五个阶段：取指；读寄存器/处理立即数/指令译码；运算；读写内存/处理跳转；写寄存器。

使用级间寄存器来存储中间状态，以便进行流水线工作。它们在时钟信号上升沿写入上个阶段准备好的数据，以进行流水线的推进。ID段和MA段涉及存储器，如果这些存储器（非级间寄存器）同样上升沿工作，时常产生数据不稳定写入错误的问题，在采用各种方法尝试降低延迟无果后，换为在下降沿写入数据，解决了问题。

CPU中所使用的各种器件尽可能来自之前五次实验，所以控制模块将指令前六位的译码和功能码（即后六位）的译码集成在一个模块里（虽然数据通路图里ALUCU在EX段但实际上在ID段），加法器如前面实验使用了**先行进位加法器**，数据存储器数据的**输入输出使用同一个接口**。

控制器：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image2.png)

在ID阶段译码（包括ALU功能码在内），使用级间寄存器将控制信号传递下去，各阶段取用各自阶段所需的控制信号。

各控制信号解释：

PCSrc：选择下一条指令地址是来自跳转还是来自当前指令加一。

DL：是否发生延迟，发生延迟时传递下去一个空指令并且PC不自增。这个信号是由冒险解决模块而非CU控制的。

RegWr：寄存器是否写。由CU译码结果和ALU运算结果共同决定。

k：是否向31号寄存器写，用于jal指令，不由CU产生而由MemToReg和CS的运算得出。因为jal指令借用了MemOut寄存器，当数据存储器不工作而保存数据存储器的寄存器工作时，一定运行了jal指令，则jal的下一条指令地址存入31号寄存器。

HL：立即数由16位扩展到32位时放在高16位还是低16位。高16位用于lui指令。

Signed：立即数放在低16位时高位是有符号扩展还是无符号扩展。有符号扩展补1还是0由立即数最高位决定。

ALUSrc、ALUASrc：决定ALU运算两个输入数据的来源。

ALUOp：4位，控制ALU的功能。

U：运算是否为带U的运算。对于加减法，U代表不考虑溢出；对于大小比较、乘除法，U代表数据视作无符号数。

JR：跳转指令是否为jr指令（而非j指令），控制跳转地址的来源。

ZF：运算结果是否为0，用于beq、bne。

CF：这里代表是否发生溢出。

Branch：是否为分支跳转指令。

Jump：是否为无条件跳转指令。

MemRW：数据存储器是读还是写。

CS：2位，00代表数据存储器不工作，01代表字节操作，10代表半字操作，11代表字操作。

SignedMem：数据存储器加载数据时视作有符号数还是无符号数，决定高位的填充。

MemToReg：选择寄存器写的数据来源是内存还是ALU计算结果。

RegDst：选择寄存器写的地址来源，是指令15-11位还是20-16位，对应r型和i型指令。

存储器：指令存储器按字存储，数据存储器按字节存储。其中数据存储器为4x4片1k字节的子存储器构成，根据控制信号决定启用一片（4个选择）、两片（2个选择）或四片，对应存取字节、半字和字。取数的有符号扩展或无符号扩展也集成在数据存储器中。

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image3.png)

加法溢出的处理：对于add、sub、addi，因本CPU未设计中断，所以对溢出的响应设计为结果不写入寄存器。另外溢出判断为：先行进位加法器中向最高位的进位和溢出位不同，则为真正的溢出。

流水线冒险：通过自动添加气泡解决。对于各种跳转类指令，必须在其后保持三个周期的空指令；对涉及写寄存器的指令，向其后检查三条指令，如果其中有读同一个寄存器的指令，则在其后保持n个周期的空指令，n由最近的发生数据冒险的指令决定（紧接着一条指令就发生数据冒险则n=3，隔一条则n=2，隔两条则n=3），另外控制PC自增器不自增。不处理寄存器地址0的指令，因为0号寄存器无论读还是写都恒为零。

另外，指令j和jal，以及内存存取指令，从MARS模拟器复制到指令存储器中时应当做一定调整：地址位的高位应当取零。这是因为在MARS中数据段和代码段地址都不是从零开始的，它们共用一个地址空间的不同段，而我们的CPU中数据存储器和指令存储器分开，地址都从零开始。但是即使不做调整也可以正常工作，这是因为我们的CPU中地址都不会识别高位，存储器本身没有足够大空间支持如此大的寻址空间（尝试了使用足够大的存储器，仿真变得非常慢，基本上仿真不出来，遂不采用）。

测试：

内存存取类测试：

lui \$a1,0x81ff

lui \$a2,0x4386

lui \$k1,0x81fe

lui \$t1,0x1001

srl \$a1,\$a1,16

or \$a2,\$a1,\$a2

sb \$a1,3(\$t1)

sh \$a1,4(\$t1)

lb \$t2,3(\$t1)

sw \$a2,8(\$t1)

lbu \$t3,3(\$t1)

lh \$t4,4(\$t1)

lhu \$t5,4(\$t1)

lw \$t6,8(\$t1)

机器码：

0x3c0581ff

0x3c064386

0x3c1b81fe

省略，相当于存放数据段基地址0

0x00052c02

0x00a63025

0xa1250003

0xa5250004

0x812a0003

0xad260008

0x912b0003

0x852c0004

0x952d0004

0x8d2e0008

运行结果：

MARS模拟器：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image4.png)

仿真：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image5.png)

r型指令测试：

lui \$a1,0x81ff

lui \$a2,0x4386

lui \$k1,0x81fe

lui \$k0,0x81fe

srl \$a1,\$a1,16

or \$a2,\$a1,\$a2

add \$t7,\$a1,\$a2

addu \$s0,\$a1,\$a2

sub \$s1,\$a1,\$a2

subu \$s2,\$a1,\$a2

sub \$k1,\$a1,\$a2

sll \$t7,\$t7,5

sllv \$s0,\$s0,\$k1

sra \$s1,\$s1,3

and \$s3,\$s1,\$s2

or \$s4,\$k1,\$t7

xor \$s5,\$a1,\$a2

nor \$s0,\$k1,\$t7

srl \$k0,\$k0,16

subu \$a1,\$a1,\$k0

srlv \$t7,\$t7,\$a1

机器码：

0x3c0581ff

0x3c064386

0x3c1b81fe

0x3c1a81fe

0x00052c02

0x00a63025

0x00a67820

0x00a68021

0x00a68822

0x00a69023

0x00a6d822

0x000f7940

0x03708004

0x001188c3

0x02329824

0x036fa025

0x00a6a826

0x036f8027

0x001ad402

0x00ba2823

0x00af7806

运行结果：

MARS模拟器：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image6.png)

仿真：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image7.png)

i型指令测试：

lui \$k0,0x81fe

addi \$t1,\$k0,123

addiu \$t2,\$k0,234

andi \$t3,\$k0,556

ori \$t4,\$k0,6890

xori \$t5,\$k0,6895

slti \$t6,\$k0,3688

sltiu \$t7,\$k0,3688

机器码：

0x3c1a81fe

0x2349007b

0x274a00ea

0x334b022c

0x374c1aea

0x3b4d1aef

0x2b4e0e68

0x2f4f0e68

运行结果：

MARS模拟器：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image8.png)

仿真：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image9.png)

溢出测试，以addi为例：

lui \$k0,0x8000

addiu \$t9,\$k0,-1

addi \$k1,\$k0,-1

机器码：

0x3c1a8000

0x2759ffff

0x235bffff

测试：

MARS模拟器（发生了中断）：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image10.png)

仿真：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image11.png)

可见k1都没有写入数据。

跳转和乘除法等综合测试：

j MAIN

FUNC:

addi \$t1,\$at,1900

mult \$t2,\$v0

mfhi \$t3

sub \$t2,\$t3,\$v0

div \$t1,\$t2

mflo \$t1

jr \$ra

MAIN:

addi \$at,\$zero,5790

lui \$t2,3457

addi \$v0,\$zero,233

jal FUNC

add \$t5,\$t1,\$t2

jal FUNC

lui \$k0,0x1001

FOR:

addi \$13,\$13,1

sb \$t1,5(\$k0)

addi \$k0,\$k0,1

bne \$13,\$9,FOR

lh \$t5,-219(\$k0)

or \$t6,\$t5,\$v0

slt \$k1,\$t2,\$t3

FOR1:

srav \$t6,\$t6,\$k1

sw \$t6,1(\$k0)

addi \$k0,\$k0,4

bne \$t3,\$t6,FOR1

机器码：

08000008

2029076c

01420018

00005810

01625022

012a001a

00004812

03e00008

2001169e

3c0a0d81

200200e9

0c000001

012a6820

0c000001

3c1a1001

21ad0001

a3490005

235a0001

15a9fffc

874dff25

01a27025

014bd82a

036e7007

af4e0001

235a0004

156efffc

测试结果：

MARS模拟器：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image12.png)

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image13.png)

仿真（数据存储器取低八位部分具有特点的内容）：

![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image14.png)
![](https://github.com/hihidhihi/XJTU_Computer_Organization_Experiment/blob/main/images/media/image15.png)

**五、调试和心得体会**

由以上测试可以看出，本CPU可以顺利实现MIPS-Lite指令子集中的所有指令，包括LB、LBU、LH、LHU、LW、SB、SH、SW、ADD、ADDU、SUB、SUBU、SLL、SRL、SRA、SLLV、SRLV、SRAV、AND、OR、XOR、NOR、ADDI、ADDIU、ANDI、ORI、XORI、LUI、SLTI、SLTIU、BEQ、BNE、J、JAL、JR、MULT、MULTU、DIV、DIVU，另外加入了SLT、SLTU、MFHI、MFLO。这些测试体现了各指令的不同功能、带U和不带U的不同结果、函数跳转返回和循环的效果，本CPU完整地完成了这些任务。

实验过程中，考虑到由单周期CPU改造成五段流水线CPU可能要改变非常多内容，直接建立五段流水线CPU。在除了教科书没有任何参考的情况下，我先按照《计算机组成与设计》内流水线CPU的示例设计图逐级拼凑，每拼凑一级就测试指令能否前进到这一步，最后达成一条指令的运转。随后一个一个地添加新的功能，用短程序测试，在此过程中反复扩大控制信号位数和内容，改造已有的器件，添加新的器件。最后进行流水线冒险的解决。

改造原有器件会造成额外的麻烦，比如RAM数据输入输出只使用一个接口，在接口内部又有一些其他功能连接，造成它们也都要慎重考虑双向接口。

实验中遇到的问题数不胜数，另外往往在一个思路下无论怎么修改都达不到要求，最后完全改变思路（比如完全不走这条数据通路）才得以解决问题。各种问题中最记忆深刻且反复出现的问题为存储器写入数据出错，最初只需减少中间寄存器的使用，降低时延即可解决，后来功能越加越多无法解决了，最终使之下降沿响应才彻底解决。此过程中我了解到虽然assign可能降低时延，但其后的式子过长就不行了，以及case语句虽然只有借助reg才能使用，但不一定真的在综合时使用reg和造成大时延。尝试用assign代替case的语句在现在的代码中尚有残留。

此外出现了一些莫名其妙的问题，比如某个32位的wire在之前的仿真中还正常，某一次后突然识别不到了，波形图中变为1位的wire且其值恒为Z，仿真不能正常进行，检查代码一切正常。各种尝试无果后，把变量名换了一个就恢复了。但在代码中绝没有出现变量名重复的错误。再比如在最后添加冒险解决模块时，仿真一直报错，最后把代码放在CPU顶层代码的同一个文件里就解决了。只是放在同一个文件中，而非同一个模块。

弄清每条指令的各位规定和运算遵循的规则十分困难，只能在网络上找到一些不完整的参考，最终是通过这些参考加上在模拟器里测试才确定的。每条指令都要测试，十分繁琐。直到实验完成才发现原来老师已经发了MIPS指令集的文档，十分遗憾。

测试程序的编写也很繁琐。直到所有实验完成，编写最后一个综合测试程序时，才发现MARS模拟器可以导出16进制机器码，在此之前大量的机器码都是从模拟器代码段一句一句复制修改来的。希望下次实验能在PPT里写下这样的注意事项。

注：机器码的使用方法：在C盘新建一个命名为mips1的文件，将机器码粘贴其中保存，再进行仿真即可。
