Header header

string msg_type        #消息类型
string Mode            #定位模式，A=自动2D/3D,M=手动2D/3D
uint16 PosStatus       #定位类型，1=未定位，2=2D定位，3=3D定位
uint32[12] PRNList     #PRN码-伪随机噪声码，第x信道正在使用的卫星PRN码编号
float64 PDOP           #PDOP综合位置精度因子，0.5-99.9
float64 HDOP           #HDOP水平精度因子，0.5-99.9
float64 VDOP           #VDOP垂直精度因子，0.5-99.9