Header header

string msg_type           #消息类型
float64 UTCTime           #UTC时间，格式为hhmmss.sss
int32 UTCDate             #UTC日期，DDMMYY格式
float64 X                 #X坐标，单位：米
string XDir               #X坐标方向,N/S
float64 Y                 #Y坐标，单位：米
string YDir               #Y坐标方向,E/W
uint16 PosStatus          #解状态， 0：无效解，1：单点定位解，2：伪距差分，3：固定解，4：浮动解
uint16 SatellitesCount    #使用卫星数量
float64 HDOP              #HDOP-水平精度因子，0.5到99.9，一般认为HDOP越小，质量越好
float64 Altitude          #海拔高，-9999.9到9999.9米
string AltitudeUnit       #高程单位，M指单位米