Header header

string msg_type       #消息类型
uint8 SolStatus       #结算状态
uint8 PosStatus       #定位状态
float64 Latitude      #纬度
float64 Longitude     #经度
float64 Altitude      #海拔高
float64 GeoidHeight   #大地水准面高度异常差值
string DatumId        #坐标系统
float64 LatStdDev     #纬度标准差
float64 LonStdDev     #经度标准差
float64 HgtStdDev     #高度标准差
string StnId          #基准值ID
float64 DiffAge       #差分龄期（以秒计）
float64 SolAge        #解算时间（以秒计）
uint16 SVs            #跟踪到的卫星数
uint16 SolSVs         #参与RTK解算的卫星数
uint16 L1SolSVs       #L1参与PVT解算的卫星数
uint16 L1L2SolSVs     #L1和L2参与PVT解算的卫星数
uint16 Reserved1      #预留
uint16 ExtSolStatus   #扩展解算状态
uint16 Reserved2      #预留
uint16 SolSigs        #参与解算的信号