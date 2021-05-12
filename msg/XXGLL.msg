Header header

string msg_type           #消息类型
float64 Latitude          #纬度ddmm.mmmm(度分)格式
string LatitudeHemi       #纬度半球N或S
float64 Longitude         #经度dddmm.mmmm(度分)格式
string LongitudeHemi      #经度半球E或W
int32 UTCDate             #UTC时间,hhmmss(时分秒)格式
string Status             #定位状态,A=有效定位,V=无效定位
string ModeInd            #模式指示，仅NMEA0183 v3.00输出，A=自主定位，D=差分，E=估算，N=数据无效