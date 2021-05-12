Header header

string msg_type           #消息类型
float64 UTCTime           #UTC时间，格式为hhmmss.sss
float64 Latitude          #纬度(-90~+90)，格式为ddmm.mmmm
string LatitudeHemi       #纬度半球，N或S
float64 Longitude         #经度(-180~+180)，格式为dddmm.mmmm
string LongitudeHemi      #经度半球，E或W
uint16 StatusIndicator    #GPS状态：0=不可用(FIX NOT valid)，1=单点定位(GPS FIX)，2=差分定位(DGPS)，3=无效PPS，4=实时差分定位（RTK FIX），5=RTK FLOAT，6=正在估算
uint16 SatellitesCount    #使用卫星数量
float64 HDOP              #HDOP-水平精度因子，0.5到99.9，一般认为HDOP越小，质量越好
float64 Altitude          #海拔高，-9999.9到9999.9米
string AltitudeUnit       #高程单位，M指单位米
float64 GeoidHeight       #大地水准面高度异常差值，-9999.9到9999.9米
string GeoidHeightUnit    #水准面高度单位，M指单位米
int64 DiffTemporal        #差分GPS数据期限(RTCM SC-104)，最后设立RTCM传送的秒数量，如不是差分定位则为空
int16 DiffStationId       #差分参考基站标号，从0000到1023 