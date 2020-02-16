# 飞控说明

## 一、飞控使用说明

天地飞接收机的7个通道分别插入飞控板上标有21 22 23 24 81 82 83的通道，31 32 33 34 41 42 43 44为8个输出通道。

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image002.jpg)

除个别机型的说明外，方向如图所示放置。

上电后3个指示灯全亮，再过0.5秒根据5和7通道确定机型，之后持续等待，直到飞控放置平稳（水平且静止）后校准传感器，持续1秒，若开始校准后的这1秒内未能保持平稳则退出校准并重新持续等待。因此如果一切顺利则上电后1.5秒校准完成，完成后3个指示灯指示飞行器类型。校准未完成则油门无法启动（固定翼）甚至无法解锁（旋翼）。校准过程非常重要，校准过程中务必保持飞控放置平稳。

垂直起降的机型解锁方式为内八字解锁（左操纵杆处于最右下，右操纵杆处于最左下），须保持2秒，左操纵杆处于最左下时即刻锁定（左手油）；固定翼机型不需要解锁。

## 二、机型说明

机型决定方式：

|   -  | 五通道 |        六通道       | 七通道 |
|  -   |   -   |          -          |   -   |
|四轴  |   低   |                    |   低   |
|垂起  |   低   |                    |   中   |
|鱼鹰  |   低   | 低为旋翼，高为固定翼 |   高   |
|纸飞机|   中   |                    |   低   |
|老祖宗|   中   |                    |   中   |
|对地  |   中   |                    |   高   |

**1.** **四轴**

通道31 32 33 34分别接前后左右电机。十字形。

**2.** **垂起**

通道31 32 33 34分别接左右电机、左右舵机。暂时按照轴的方式进行控制，以后可能会改成吊机的方式。飞控和导线等面向自己，PWM增加时，左舵机往里，右舵机往外，俯视顺时针。飞控前后反向。

**3.** **鱼鹰**

通道31 32 33 34分别接左右电机、左右倾转舵机，通道41 42 43 44分别接副翼、升降舵、空、方向舵。PWM增加时，左舵机往后，右舵机往前，俯视逆时针。

7通道高挡和中挡为旋翼模式（测试通过），低挡为固定翼模式（未测试）。

飞行教学：

上电后为旋翼模式，与旋翼的控制方式相同；7通道为低档时与固定翼控制方式相同，与固定翼的飞行方式的区别为：横滚不用副翼而靠电机舱的角度控制，

**4.** **纸飞机**

通道31 32 33分别接副翼、升降舵、电机。PWM增加时，左舵机往下，右舵机往上，右偏。

**5.** **老祖宗**

通道31 32 33 34分别接副翼、升降舵、电机、方向舵。自动模式有自稳功能。副翼反向。

**6.** **对地**

通道31 32 33 34分别接副翼、升降舵、电机、方向舵，通道41 42分别接云台横向舵机和纵向舵机。副翼、平尾、垂尾均反向。云台横向舵机反向。

## 三、程序说明

与stm32底层驱动有关的c文件都在bsp文件夹中，与传感器和姿态解算有关的都在sensor文件夹中，其余的在user文件夹。每个文件include其他文件都在对应的h文件中，尽量减少耦合也便于移植。

程序架构的思想是使用定时任务的标志位寄存器，在systick定时中断中每计时满某一个定时周期就置位相应的标志位，在main函数的死循环中如果检测到相应标志位置位就执行相应定时周期的任务，执行完后将标志位清零。总共有6个定时周期。由滴答定时器产生1ms定时，1ms定时中断中喂狗，中断中计数产生周期不同的循环任务。定时中断在main.c中。

所有自定义的标志位寄存器都为unsigned char型，变量定义在c文件中，寄存器各个位为宏定义，定义在对应的h文件中（main.c例外）。

主要代码都在task.c中，各种飞行器机型的控制程序在control.c中。机型之间的6处区别为：PID等参数，电机内环(20ms)，电机外环(50ms)，舵机内环(50ms)，电机锁定(100ms)，舵机外环(200ms)。

与地面站的通信在niming.c中。串口发送用DMA，接收仍然单字节处理

姿态解算用的四元数转欧拉角和互补滤波等算法，直接处理mpu6050的原始数据，各种数学运算在mymath.c中，没用标准C语言math.h

电机控制采用完整的PID控制，内外环PID分别使用一个结构体且两者定义相同；舵机控制采用双P控制，只用一个结构体且与电机的结构体不同，内外环只用单独的P，而且两个P值的乘积应为1，一般来说内环的P值应小于1，该值越大，舵机的自动调整越剧烈。

接收机输入捕获用定时器TIM2和TIM8，7个通道，输出用TIM3和TIM4。（通道按顺序排列）

上电后指示灯亮，初始化和自检结束后灯灭。

PID参数有待优化。

## 四、算法详细说明

**1.** **PID**

参考simulink框图。

不完全微分：反馈支路增益N在(0,1)之间，越小则滤波越明显，为0时无微分，为1时为完全微分，参考值0.5。上述为离散情况，若为连续情况，同样越小则滤波越明显，参考值5。

积分限幅：积分限幅值的估算方法为：积分系数按0.1计算，误差按积分分离的阈值（10°）计算，积分运算时间不应超过5秒，角度限幅值与角速度限幅值之比为其函数执行频率之比（2:5），即角度限幅值为10\*0.1\*20*5=100，角速度为250。以角度为例，达到限幅值后积分项输出为10，与比例系数为1时的比例项输出相等。

**2.** **数据校准**

**3.** **卡尔曼滤波**

矢量状态—标量观测卡尔曼滤波

状态方程：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image004.gif)

 **s**\[n\] **A** **s**\[n-1\] **B** **u**

观测方程：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image006.gif)

x ![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image008.gif) **s**

预测：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image010.gif)

最小预测MSE：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image012.gif)

卡尔曼增益：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image014.gif)

修正：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image016.gif)

最小MSE：

![](file:///C:/Users/mynam/AppData/Local/Temp/msohtmlclip1/01/clip_image018.gif)

其中

方差的某次室内环境下的实测数据：

加速度计：roll 0.0225；pitch 0.006

陀螺仪：roll 0.0494；pitch 0.0038；yaw 7.0264e-04

## 五、变量命名规则

函数名的每个单词开头大写，用下划线分隔；

变量名（包括结构体名）若只有一个单词则小写，若有多个单词则每个单词开头大写且不用下划线；

宏定义单个单词则全部大写，多个单词则全部大写并用下划线分隔；

特殊情况例外。

## 六、版本说明

程序版本：2.07

版本号格式：主版本号.子版本号（经过飞行检验）.阶段版本号。

文档版本：2.07

## 七、bug及其他注意事项

1. 数传波特率有限，发送数据过多会导致复位，从而有工作过程中更换机型的风险。经测试打开第7通道允许发送后的3条发送语句为极限，为安全起见，最多允许2条发送语句。若用半双工数传则不存在这个问题。

2. 四轴模式下调PID时写入P值超过1000会使油门瞬间达到满量程并恢复正常，原因未知。

3. 不建议在数据融合前对原始数据使用滑动平均滤波。

4. 对地机型可能有安全隐患，炸机原因未知。

## 八、其它记录

**4.** **PWM**

天地飞接收机电压5v，PWM信号高电平3.4v，周期21.20ms，频率47.17hz，脉宽1ms~2ms,占空比4.7%~9.4%。1,000,000/47.17=21200

**5.** **舵机行程**

理论极限：周期20000，最小行程500（2.5%），归中行程1500（7.5%），最大行程2500（12.5%）

天地飞：周期21200，最小行程1000（4.7%），归中行程1500（7.1%），最大行程2000（9.4%）

实际极限：周期21200，最小行程530（2.5%），归中行程1590（7.5%），最大行程2650（12.5%）

推荐极限：周期21200，最小行程590（2.8%），归中行程1590（7.5%），最大行程2590（12.2%）

天地飞在开三角翼混控时输出可达到理论极限。

**6.** **MPU6050****用法**

俯仰角（pitch）横滚角（row）航向角（yaw）

MPU6050模块水平放置，排针朝下朝右，向左横滚roll和gyrox为正,accy为正；抬头pitch和gyroy为正，accx为负；向左偏航yaw和gyroz为正（这3个标准也适用于天地飞遥控器初始误差补偿，以及天地飞遥控器）；偏航行程(-360,360)，横滚和俯仰行程(-180,180)。

初始化时必须把模块放正。

AD0引脚在模块内部下拉，地址默认0x68不用改。只连4个引脚即可。

# 西电航协研发部Github组织库规范
## 一．命名规范

1.库名中不得出现下述规定的字符- \ @ ! # $ % ^ & * () [] {} | \ ; : '' ’ ， 。 《 》 < > · ~ 。

2.库名应尽量避免使用名.名的形式。

3.库名应尽量使用英文，禁止使用中文字符。一般情况下，库名中出现的第一个单词的首字母应使用大写。各个单词之间空格隔开即可。

4.缩写的单词一律使用大写，如：UAV。

## 二．README规范

1.每个子储存库根目录都要有README文件，文件名统一为README.md，禁止使用其他格式的文件作为README。

2.README.md整体包含三部分：标题、目录、正文。标题为一级标题。目录、正文、正文章节为二级标题。章节内小节为三级标题，加两个缩进。

3.README.md文件编写语言只能使用markdown语言。使用其他语言编写会导致git存储库无法识别或乱码。

## 三．存储库维护规范

1.存储库中master分支为确定无误后的代码，测试代码请新建一个新的branch，勿随意合并分支中的内容。

2.若一个存储库由多人维护，每个人应有自己的分支，最终确定无误后，每个人整理自己贡献的部分，合并进master分支。

3.若一个大存储库中包含多个工程，请将这些工程分别新建存储库，之后通过submodule的方式来添加进大存储库。

## 四．Issue以及Pull request规范

1.对代码有任何疑问或者是任何建议或bug report请移步至issue栏目

2.issue中请详细描述你遇到的问题，最好附上ERROR提示代码（计算机生成的代码请用’’’ ‘’’注释）

3.请求合并时，点击“拉取请求”选项卡，然后从“拉取请求”页面中，单击绿色“新拉”请求按钮。在“示例比较”框中，选择您创建的分支，以与master（原始）进行比较.在比较页面上查看差异中的差异，确保它们是您要提交的内容。在比较页面上查看差异中的差异，确保它们是您要提交的内容。

4.Issue栏目评论时，请文明发表言论，勿辱骂他人，或刻意使用攻击性词语。
