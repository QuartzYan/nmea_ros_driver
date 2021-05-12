Header header

string msg_type               #消息类型
uint16 ItemCount              #GSV语句的总数，1~3
uint16 ItemSequence           #本句GSV的编号，1~3
uint16 SatellitesCount        #可见卫星的总数，00~12，前面的0也将被传输
uint16 PRNCode                #PRN伪随机噪声码，前面的0也将被传输 GPS:1-32,Beidou:1-37,GLONASS:1-24,Galileo:1-36,...
float32 SatelliteElevation    #卫星仰角，00~90度，前面的0也将被传输
float32 SatelliteAzimuth      #卫星方位角，000~359度，前面的0也将被传输
float32 SignalNoiseRatio      #信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输
uint16 PRNCode2               #PRN伪随机噪声码
float32 SatelliteElevation2   #卫星仰角，00~90度，前面的0也将被传输
float32 SatelliteAzimuth2     #卫星方位角，000~359度，前面的0也将被传输
float32 SignalNoiseRatio2     #信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输
uint16 PRNCode3               #PRN伪随机噪声码
float32 SatelliteElevation3   #卫星仰角，00~90度，前面的0也将被传输
float32 SatelliteAzimuth3     #卫星方位角，000~359度，前面的0也将被传输
float32 SignalNoiseRatio3     #信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输
uint16 PRNCode4               #PRN伪随机噪声码
float32 SatelliteElevation4   #卫星仰角，00~90度，前面的0也将被传输
float32 SatelliteAzimuth4     #卫星方位角，000~359度，前面的0也将被传输
float32 SignalNoiseRatio4     #信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输