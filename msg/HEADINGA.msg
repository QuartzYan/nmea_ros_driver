Header header

string msg_type           #消息类型
uint8 SolStatus           #结算状态
uint8 PosStatus           #定位状态
float64 BaseLen           #基线长，单位米
float64 Yaw               #方位角，0~360
float64 Pitch             #俯仰角，-90~90
float64 Reserved1         #预留
float64 YawStdDev         #方位角标准差，单位度
float64 PitchStdDev       #俯仰角标准差，单位度
string StnId              #基准值ID
uint16 SVs                #跟踪到的卫星数
uint16 SolSVs             #参与RTK解算的卫星数
uint16 OBs                #截止高度角以上的卫星数
uint16 L2OBs              #截止高度角以上跟踪到L2的卫星数
uint16 Reserved2          #预留
uint16 ExtSolStatus       #扩展解算状态
uint16 Reserved3          #预留
uint16 SolSigs            #参与解算的信号