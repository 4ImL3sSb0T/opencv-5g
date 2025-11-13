#ifndef CAR_Image_HH_
#define CAR_Image_HH_
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define uint8  unsigned char 
#define uint16 unsigned int   
#define uint32 unsigned long  
#define int8  char  
#define int16 int   
#define int32 long  
#define ROW  96 - 20  
#define COL 640   
#define MIDVALUE 320/2  
#define BIANJIEadd 8   //边界补线误差（差的多补的弯道比较好，默认是2）、、、、、、、、、、、、、、会不会v和
extern int find_XYdata_second[3];//障碍物坐标
void printQ(const std::string& str, int number);
using namespace std;
using namespace cv;
extern int find_XYdata[3];//障碍物坐标
extern int BS_BZ_FLAG;//避障变速标志
extern int MUSIC_FLAGG;//语音识别标志
extern int UI_music;            
extern int stopbanma; 
// void changetoright();//右换道
// void changetoleft();//左换道



extern int banmaenable;//2是不允许，1是允许
extern int bizhangenable;
extern int yellowenable;
extern int cone_guidance_enable;
void setConeGuidanceMode(int enable);
//=====>>>>>>>>>>>>>
int16 Limit_Protect(int16 num, int32 min, int32 max);
int16 Fit_Point(uint8 i, float Ka, float Kb);
char Error_Transform(uint8 Data, uint8 Set_num);//
int Q_jdz(int A);
void Curve1_Fitting(float* Ka, float* Kb, int16* Start, int16* Line_Add, int16 Mode);
void Curve2_Fitting(float* Ka, float* Kb, uint8 Start, uint8 End, int16* Line, int16 Mode, int16 num);
void Curve3_Fitting(float* Ka, float* Kb, uint8 Start, uint8 End, int16* Line, int16 Mode);//环岛检测专用，最正规的求线
void Earge_Search_Mid(int16 i, cv::Mat data, int16 Mid, int16 Left_Min, int16 Right_Max, int16* Left_Line, int16* Right_Line, int16* Left_Add_Line, int16* Right_Add_Line, int mods);
int16 First_Line_Handle(cv::Mat data);
void Mid_Line_Repair(void);//中线修复;
int Image_Handle22(cv::Mat data, cv::Mat YUANTU, cv::Mat BANMA);  //图像320 *120
//图像320 *120
int error_get(void);
int BZ_chuli(cv::Mat BZdata);//传入原图
void bizhangBuxian(int data_X, int data_Y, int bizhang_fangxiang);//0 向左避障  1向右避障
void huandaoBuxian(int changedata_X, int changedata_Y, int bizhang_fangxiang);//0 向左避障  1向右避障备份

int BZ_PANDUAN_2(void);//判断障碍物是否在赛道内
void BZ_LuoJISET(void);//避障逻辑
int TUxiang_Init(cv::Mat data);//图像预处理
void pidcont(int Error);//控制
int Biansu(void);//变速设置
int yellow_chuli(cv::Mat BZdata);
int BanMa_Find(cv::Mat BanMa_Find_data);
void car_speed_set(int data);//车速控制
int Q_jdz(int A);
int CAR_STOP(cv::Mat yellow_Find_data);
extern uint8 Weight_th[110];
extern int BMGet;//斑马线标志
extern int BZ_FLAG_FLAG;
extern int CAR_STOP_FLAG;
extern int BZget;
extern int banmaxian_Y;//斑马线坐标
extern int TIMEDELAY; //定时器延时
#define erroe_xiuzheng 0   //误差修正

#define setspeed1  2000
#define setspeed2  2500
#define setspeed3  3000
#define setspeed4  3500
#define setspeed5  4000

extern int bizhanground;
//速度分段PID
extern int car_speed;  //避障偏移量   // 3800   
//#define car_speed setspeed1  //避障偏移量   // 3800   

//参数宏定义
#define Bizhang_line_move 	20  //避障偏移量
#define blue_check_r 35    //蓝色检测半径 s = blue_check_r *blue_check_r
#define blue_check_num 35*35*0.3    //蓝色检测半径内的数目
//=====>>>>>>>>>>>>>
void onTrackbar(int, void*);
void UI_init(void);


extern cv::Mat red_mask;



extern int Hmin, Hmax, Smin, Smax, Vmin, Vmax;
extern int BZ_Imageflag, BM_Imageflag,yellow_Imageflag,blue_Imageflag,XUNJI_Imageflag,CA_Imageflag;
extern int SPEED_CARSETETETE,car_break;
extern int IMfr;//图像帧率
extern int UI_KP,UI_KD;
extern int TIME_BIZHANG;//到时间后打开臂章并减速





void page_get(cv::Mat image,int coordinates[]);
extern int hmin_white,smin_white,vmin_white,hmax_white,smax_white,vmax_white,Page_Findflag,White_Imageflag,thre;
extern int white_coordinates[8];

/*2024y*/
int TUxiang_Init3(cv::Mat data);//图像预处理
void delay_ms(int ms);
extern int SPEED_SETVALUE;//速度设置
int ABCIMAGE(cv::Mat BZdata);//传入原图
extern cv::Mat frame;
extern char GETYOLOflag;
extern int car_l_mid_stop;




#endif

