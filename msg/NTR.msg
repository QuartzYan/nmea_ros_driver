Header header

string msg_type       #消息类型
float64 UTCTime       #UTC时间，格式为hhmmss.sss
uint16 PosStatus      #解状态， 0：无效解，1：单点定位解，2：伪距差分，4：固定解，5：浮动解
float32 Distance      #距离参考站斜距，格式dddd.ddd，单位：米
float32 DistanceX     #X方向平距，+表示在参考站北方向，-表示在参考站南方向
float32 DistanceY     #Y方向平距，+表示在参考站东方向，-表示在参考站西方向
float32 DistanceZ     #Z方向平距，+表示在参考站上方向，-表示在参考站下方向
uint16 STNID          #参考站号