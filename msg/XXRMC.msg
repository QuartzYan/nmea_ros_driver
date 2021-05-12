Header header

string msg_type           #消息类型
float64 UTCTime           #UTC时间，hhmmss.sss格式
string Status             #状态，A=定位，V=未定位。当给GPS复位时为V，不输出速度，角度，时间数据 
float64 Latitude          #纬度ddmm.mmmm，度分格式
string LatitudeHemi       #纬度N或S
float64 Longitude         #经度dddmm.mmmm，度分格式
string LongitudeHemi      #经度E或W
float32 SpeedKnots        #速度，节，Knots
float32 Azimuth           #方位角，度 Azimuth/bearing
int32 UTCDate             #UTC日期，DDMMYY格式
float32 MagVar            #磁偏角，000-180度
string MagVarDir          #磁偏角方向，E或W
string ModeInd            #模式指示,仅NMEA0183 v3.00输出，A=自主定位，D=差分，E=估算，N=数据无效