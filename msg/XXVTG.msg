Header header

string msg_type                 #消息类型
float32 MovementAngleTN         #运动角度，000 - 359，以真北为参考基准的地面航向，前面的0也将被传输
string TrueNorthRef             #T=真北参照系
float32 MovementAngleMN         #运动角度，000 - 359，以磁北为参考基准的地面航向，前面的0也将被传输
string MagneticNorthRef         #M=磁北参照系
float32 HorizontalMoveSpeedKn   #水平运动速度，000.0~999.9节，前面的0也将被传输
string SpeedKnots               #N=节，Knots
float32 HorizontalMoveSpeedKm   #水平运动速度，0000.0~1851.8Km/h，前面的0也将被传输
string SpeedKmh                 #K=公里/时，km/h
string ModeInd                  #模式指示,仅NMEA0183 v3.00输出，A=自主定位，D=差分，E=估算，N=数据无效