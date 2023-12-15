/*
 * - EN1, EN2, EN3 → UL, VL, WL
 * - PWM優先駆動、PI制御周期化、アクセル⇔ボリューム切り替え機能追加
 * - 速度制御、電流制御、PWMDuty制御 切り替え機能追加
 */
// just=0.7;        // Nrpm_S回転速度計算値の調整

#include "mbed.h"
#include <math.h>

#define VMAX_SET  1.0       // PI 制御最大電圧指令 0.0 〜 1.0
#define IMAX_SET  0.4       // PI 制御最大電流指令 0.0 〜 1.0

/*
 * PI制御周期設定
 * コメントアウトでPI周期制御無し
 */
#define TickSpeed
#define TickCurrent

PwmOut PWM_U(PA_8); 
PwmOut PWM_V(PA_9); 
PwmOut PWM_W(PA_10);

DigitalOut UL(PA_7);
DigitalOut VL(PB_0);
DigitalOut WL(PB_1);

AnalogIn Curr_ui(PA_0);
AnalogIn Curr_vi(PC_1);
AnalogIn Curr_wi(PC_0);

// AnalogIn V_adc(PB_1); // Blue On Board volume
// AnalogIn V_adc(PC_2); // Gaibu Potention IHM07
// AnalogIn V_adc(PA_6); // Gaibu Potention IHM08

//
// AnalogIn 0.0 ~ 1.0 (1.0 means 3.3V)
//
AnalogIn V_adc(PC_2);

InterruptIn  HA(PA_15);
InterruptIn  HB(PB_3);
InterruptIn  HC(PB_10);

DigitalIn  U_in(PA_15);
DigitalIn  V_in(PB_3);
DigitalIn  W_in(PB_10);

DigitalIn Direction(PB_8);

Timer uTimer;
Timer vTimer;
Timer wTimer;
Timer Timer1;

AnalogOut SWAVE(PA_4);

Serial pc(USBTX,USBRX);

// つかってない
// DigitalOut myled(LED1);  

// オプション
unsigned char   acc_vol = 1;                            // オプション１ acc_vol==0 はVolume  acc_vol==1はAccel
unsigned char   Sp_tick = 0, Cu_tick = 0;               // 周期PI制御フラグ
float           Sp_ticktime = 160, Cu_ticktime = 80;    // オプション２，３ PI制御周期時間、単位μsec
float           change_pwm_N = 20;                      // オプション５ ホールセンサエッジPWM優先駆動開始回転数
unsigned char   rt_pwm;                                 // 1: ホールセンサエッジ割り込み PWM 優先駆動フラグ, 0: 優先無し

unsigned char  HUVW;                                    // サンプリング開始時の Hole IC の状態
unsigned int t_cnt = 0;                                 // キャプチャ間の cnt 数
unsigned int Timer_cnt_C = 0;                           // キャプチャの値
unsigned int Timer_cnt_A = 0;                           // キャプチャの値
unsigned int Timer_cnt_B = 0;                           // キャプチャの値
unsigned int Timer_cnt_C_1 = 0;                         // キャプチャの値 old
unsigned int Timer_cnt_A_1 = 0;                         // キャプチャの値 old
unsigned int Timer_cnt_B_1 = 0;                         // キャプチャの値 old

float   iu_ad, iv_ad, iw_ad;
float   Vu_ad, Vv_ad, Vw_ad;
float   vr1_ad = 0, vr_ad, vr1_ad_p; // vr2_ad;
float   Vr_adc_i;  
float   refu, refv, refw;
float   curr_ub, curr_vb, curr_wb, curr_u, curr_v, curr_w;

float   Nrpm = 0;                                       // 回転数[r/min]
float   Nrpm_Previous = 0;                              // 回転数[r/min] old
float   Nrpm_s = 0;                                     // 回転数[r/min]
float   Speed_now = 0;
float   Speed_MAX = 3500.0;                             // 速度指令合大値 6直 1500, 3直 2並 2500, 2直 3並 3500, 6並 6500
float   Speed_SET = 0.0;                                // 速度指令[r/min]
float   Speed_SET0 = 0.0;                               // 速度指令[r/min]
float   Accel_dN = 1.0;                                 // 加速レイト
float   Decel_dN = 1.0;                                 // 減速レイト
float   Speed_mini = 300;
float   Speed_err_MAX = 10000;

/*
 * 正弦波駆動に向けて追加
 */
unsigned int Timer_FLG = 0;                             // キャプチャあり/なし, 1/0
unsigned int Timer_cnt_start = 0;                       // サンプル開始時のカウンタ値
unsigned int Timer_cnt_Hole = 0;                        // キャプチャ時ののカウンタ値

unsigned char  UVW;
unsigned char  U, V, W;
unsigned char  PWM_Drive_M = 1;                         // PWMモード
unsigned char  Direct_R = 1;                            // 1:正転, 0:逆転

/*
 * 電流制御
 */
float kpCurrent = 0.000522;                             // 比例ゲイン
float kiCurrent = 0.00022;                              // 積分ゲイン
float s_kiCurrent = 0;
float I_PI = 0;
float I_PII;
float I_diff = 0;
float I_detect = 0;
float Vpi = 0;

/*
 * 速度制御
 */
float kaiten = 0;
float Speed_diff = 0.0;                                 // 速度誤差
float kpSpeed = 1.4;                                    // 比例ゲイン 0.4
float kiSpeed = 0.8;                                    // 積分ゲイン 0,2
float s_kiSpeed = 0.0;                                  // 積分器中身
float Speed_diff_norm = 0.0;
float Ajust=0.7;                                        // Nrpm_S 回転速度計算値の調整

float  PWMDuty = 0;

/*
 * Ticker
 */
Ticker Sp;
Ticker Cu;

/*
 * Hall sensor
 */
uint8_t UVW_in(void)
{
    return (uint8_t) ((W_in << 2) | (V_in << 1) | U_in);
}

/*
 * Hall Caputure
 */
void Capture_u()
{            
    Timer_cnt_A = uTimer.read_us();
    Timer_FLG = 1;                          // キャプチャ発生フラグ
    Timer_cnt_Hole = Timer_cnt_A;

    t_cnt = Timer_cnt_A - Timer_cnt_A_1;

    if (t_cnt < 1) {
        t_cnt = 1;
    } else {
        // nop;
    }

    Nrpm = (float)(5000000 / t_cnt);

    if (((Nrpm - Nrpm_Previous) < Speed_err_MAX) && ((Nrpm - Nrpm_Previous) > -Speed_err_MAX)) {  // abnormal speed
        Nrpm_Previous = Nrpm;
    } else {
       Nrpm = Nrpm_Previous;
    }
    Timer_cnt_A_1 = Timer_cnt_A;
}
    
void Capture_v()
{
    Timer_cnt_B = vTimer.read_us();
    Timer_FLG = 1;                          // キャプチャ発生フラグ
    Timer_cnt_Hole = Timer_cnt_B;

    t_cnt = Timer_cnt_B - Timer_cnt_B_1;
    
    if (t_cnt < 1) {
        t_cnt = 1;
    } else {
        // nop;
    }
    
    Nrpm = (float)(5000000 / t_cnt);

    if( ((Nrpm-Nrpm_Previous) < Speed_err_MAX) && ((Nrpm - Nrpm_Previous) > -Speed_err_MAX) ) { // abnormal speed
        Nrpm_Previous = Nrpm;
    } else {
       Nrpm = Nrpm_Previous;
    }
    Timer_cnt_B_1 = Timer_cnt_B;
}

void Capture_w()
{
    Timer_cnt_C = wTimer.read_us();
    Timer_FLG = 1;                          // キャプチャ発生フラグ
    Timer_cnt_Hole = Timer_cnt_C;

    t_cnt = Timer_cnt_C - Timer_cnt_C_1;

    if (t_cnt < 1) {
        t_cnt = 1;
    } else {
        // nop;
    }
    Nrpm = (float)(5000000 / t_cnt);
    if( ((Nrpm - Nrpm_Previous) < Speed_err_MAX) && ((Nrpm - Nrpm_Previous) > (-Speed_err_MAX)) ) { // abnormal speed
        Nrpm_Previous = Nrpm;
    } else {
       Nrpm = Nrpm_Previous;
    }
    Timer_cnt_C_1 = Timer_cnt_C;
} 

/*
 * Hall_u に キャプチャ発生
 */
void Hall_u()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0); 
            UL = 1; VL = 0; WL= 1;                  // 0: active 1: high inpeedance 
        }
        if (Direct_R==1) {
            PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty); 
            UL = 1; VL = 0; WL= 1;
        }
    } else {
        // nop;
    }
    Capture_u();
}

void Hall_ul() 
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0); 
            UL = 0; VL = 1; WL = 1;
        }
        if (Direct_R == 1) {
            PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0); 
            UL = 1; VL = 1; WL = 0;
        }
    } else {
        // nop;
    }  
    Capture_u();
}  

/*
 * Hall_v にキャプチャ発生
 */
void Hall_v()
{ 
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0); 
            UL = 1; VL = 1; WL = 0;
        }
        if (Direct_R == 1) {
            PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0); 
            UL = 1; VL = 1; WL = 0;
        }
    } else {
        // nop;
    } 
    Capture_v();
}

void Hall_vl() 
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty); 
            UL = 1; VL = 0; WL = 1;
        }
        if (Direct_R == 1) {
            PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty); 
            UL = 0; VL = 1; WL = 1;
        }
    } else {
        // nop;
    }
    Capture_v();
}      

/*
 * Hall_Wにキャプチャ発生
 */
void Hall_w()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty); 
            UL = 0; VL = 1; WL = 1;
        }
        if (Direct_R == 1) {
            PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0); 
            UL = 0; VL = 1; WL = 1;
        }
    } else {
        // nop;
    }
    Capture_w();
}

void Hall_wl() 
{ 
    if (rt_pwm == 1) {
        if (Direct_R == 0) {  
            PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0); 
            UL = 1; VL = 1; WL = 0;
        }
        if (Direct_R == 1) {  
            PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0); 
            UL = 1; VL = 0; WL = 1;
        }
    } else {
        // nop;
    }
    Capture_w();
}

/*
 * SpeedのPI制御
 */
void Speed_PI()
{
    s_kiSpeed += kiSpeed * Speed_diff;
    if (s_kiSpeed > IMAX_SET) {
        s_kiSpeed = IMAX_SET;
    } else {
        if (s_kiSpeed < -IMAX_SET) {
            s_kiSpeed = -IMAX_SET; 
        } else {
            // nop;
        }
    }
  
    I_PI = s_kiSpeed + (kpSpeed * Speed_diff);
    if (I_PI > IMAX_SET) {
        I_PI = IMAX_SET;
    } else {
        if(I_PI < -IMAX_SET) {
            I_PI = -IMAX_SET; 
        } else {
            // nop;
        }
    }
}

/*
 *
 */
void Current_PI()
{
    I_diff = (I_PI - (I_detect - 0.5)) * I_PII;
    s_kiCurrent += kiCurrent * I_diff;

    if (s_kiCurrent > VMAX_SET) {
        s_kiCurrent = VMAX_SET;
    } else {
        if (s_kiCurrent < -VMAX_SET) {
            s_kiCurrent = -VMAX_SET; 
        } else {
            // nop;
        }
    }

    Vpi = s_kiCurrent + (kpCurrent * I_diff);
    
    if (Vpi > VMAX_SET) {
        Vpi = VMAX_SET;
    } else {
        if (Vpi < -VMAX_SET) {
            Vpi = -VMAX_SET; 
        } else {
            // nop;
        }
    }
}

/*
 * main
 */    
int main()
{
    Timer1.start();  
    uTimer.start();
    vTimer.start();
    wTimer.start();
      
    PWM_U.period_us(25);
    PWM_V.period_us(25);
    PWM_W.period_us(25);
    
    pc.baud(9600);

    wait_ms(500);    
    
    Vr_adc_i = V_adc.read();    // 最初のポジション

#ifdef TickSpeed
    Sp.attach_us(&Speed_PI, Sp_ticktime);
    Sp_tick = 1;
#endif

#ifdef TickCurrent
    Cu.attach_us(&Current_PI, Cu_ticktime);
    Cu_tick = 1;
#endif
      
    while (1) {
        // wait_us(50);
        // pc.printf("%.3f,%.3f,%.3f \r" ,du,dv,dw);
        vr_ad = V_adc.read();   // ループに入ってからのポジション

        pc.printf("vr_ad:%.3f, Dir:%d\r\n", vr_ad, Direction.read());

        vr1_ad_p = (vr_ad - Vr_adc_i);              // ボリュームを使う場合
        // vr1_ad_p = (vr_ad-Vr_adc_i) * 1.3;       // カート・キットのアクセルを使う場合
        vr1_ad += (vr1_ad_p - vr1_ad) * 0.2;         // 0.1

        Timer_cnt_start = uTimer.read_us();         // カウンタ値

        if (fabs(vr1_ad) < 0.1f) {
            Speed_SET = Speed_SET0 = Speed_now=0;
            I_PI = I_diff = Speed_diff = 0;
            Nrpm_s = Nrpm = 0;
            PWMDuty=0;

            Timer_cnt_C = 0;        // キャプチャの値
            Timer_cnt_A = 0;        // キャプチャの値
            Timer_cnt_B = 0;        // キャプチャの値
            Timer_cnt_C_1 = 0;      // キャプチャの値 old
            Timer_cnt_A_1 = 0;      // キャプチャの値 old
            Timer_cnt_B_1 = 0;      // キャプチャの値 old

            s_kiCurrent = s_kiSpeed = 0;
            I_PI = I_diff = I_detect = 0;

            Vpi = 0;
            kaiten = 0;
            rt_pwm = 0;
        } else {   
            // nop;
        }

        // いっしょじゃん！！！
        if (Direct_R > 0) {
            curr_ub = Curr_ui.read();
            curr_vb = Curr_vi.read();
            curr_wb = Curr_wi.read();
        } else {
            curr_ub = Curr_ui.read();
            curr_vb = Curr_vi.read();
            curr_wb = Curr_wi.read();
        }
        curr_u += (curr_ub - curr_u) * 0.1;
        curr_v += (curr_vb - curr_v) * 0.1;
        curr_w += (curr_wb - curr_w) * 0.1;

        /*
         * [ 1 ] 速度検出
         */
        /* キャプチャ発生 */
        // ここもいっしょじゃん！
        if (Direct_R == 1) {
            HA.rise(&Hall_u);
            HA.fall(&Hall_ul);
            HB.rise(&Hall_v);
            HB.fall(&Hall_vl);
            HC.rise(&Hall_w);
            HC.fall(&Hall_wl);
        }
        if (Direct_R == 0) {
            HA.rise(&Hall_u);
            HA.fall(&Hall_ul);
            HB.rise(&Hall_v);
            HB.fall(&Hall_vl);
            HC.rise(&Hall_w);
            HC.fall(&Hall_wl);
        }
        Nrpm_s = Nrpm;

        /*
         * [ 2 ] 速度制御
         */
        Speed_SET0 = fabs(Speed_MAX * vr1_ad); 
        if (Speed_SET0 > 0) {      
            if (Speed_now > Speed_SET0) {
                Speed_now -= Decel_dN; 
                if (Speed_now < Speed_SET0) {
                    Speed_now = Speed_SET0;
                } else {
                    // nop;
                }
            } else {
                // nop;
            }
            if (Speed_now < Speed_SET0) {
                Speed_now += Accel_dN;  
                if (Speed_now > Speed_SET0) {
                    Speed_now = Speed_SET0;
                } else {
                    // nop;
                }
            } else {
                // nop;
            }
            if (Speed_now < Speed_mini) {
                Speed_now = Speed_mini;
            } else {
                // nop;
            }
        } else {
            // nop;
        }

        Speed_SET = Speed_now; 
        Speed_diff = Speed_SET - (Nrpm_s * Ajust);
        Speed_diff_norm = Speed_diff / Speed_MAX;       //正規化
         
        /*
         * Speed の PI制御
         */
        if (Sp_tick == 0) {       
            s_kiSpeed += kiSpeed*Speed_diff_norm;
            if (s_kiSpeed > IMAX_SET) {
                s_kiSpeed = IMAX_SET;
            } else {
                if (s_kiSpeed < -IMAX_SET) {
                    s_kiSpeed = -IMAX_SET; 
                } else {
                    // nop;
                }
            }

            I_PI = s_kiSpeed + (kpSpeed * Speed_diff);

            if (I_PI > IMAX_SET) {
                I_PI = IMAX_SET;
            } else {
                if (I_PI < -IMAX_SET) {
                    I_PI = -IMAX_SET; 
                } else {
                    // nop;
                }
            }
        } else {
            // nop;
        }   
        /*
         * [ 3 ] 電流制御
         */
        /*
         * Current Read
         */
        if (Direct_R == 0) {
            switch (PWM_Drive_M) {
            case 1: I_detect = curr_v; break;
            case 2: I_detect = curr_w; break;
            case 3: I_detect = curr_w; break;
            case 4: I_detect = curr_u; break;
            case 5: I_detect = curr_u; break;
            case 6: I_detect = curr_v; break;
            default: I_detect = 0.0;
            }
        }
       if (Direct_R == 1) {
            switch (PWM_Drive_M) {
            case 1: I_detect = curr_w; break;
            case 2: I_detect = curr_w; break;
            case 3: I_detect = curr_v; break;
            case 4: I_detect = curr_v; break;
            case 5: I_detect = curr_u; break;
            case 6: I_detect = curr_u; break;
            default: I_detect = 0.0;
            }
        }
        /*
         * CurrentのPI制御
         */
        if (Cu_tick == 0) {
            I_diff = (I_PI - (I_detect - 0.5)) * I_PII;
            s_kiCurrent += kiCurrent * I_diff;
            if (s_kiCurrent > VMAX_SET) {
                s_kiCurrent = VMAX_SET;
            } else {
                if (s_kiCurrent < -VMAX_SET) {
                    s_kiCurrent = -VMAX_SET; 
                } else {
                    // nop;
                }
            }
            Vpi = s_kiCurrent + kpCurrent * I_diff;
            if (Vpi > VMAX_SET) {
                Vpi = VMAX_SET;
            } else {
                if (Vpi < -VMAX_SET) {
                    Vpi = -VMAX_SET; 
                } else {
                    // nop;
                }
            }
        } else {
            // nop;
        }

        kaiten = fabs(Nrpm_s);

        // OOPTION4
        /********** Speed  Current  PWMDuty Control *********/
        I_PII = 1.0;                        // Speed or Duty Control
        // I_PII = fabs(vr1_ad);            // Current Control
        // PWMDuty = Vpi;                   // 0.381667 Speed or Current Control
        PWMDuty = fabs(vr1_ad) * Vpi;       // Duty Control

        if (kaiten < change_pwm_N) {
            rt_pwm=0;
        } else {
            rt_pwm=1; 
        }


#if 0
        // こっちにはいかない
        if (acc_vol == 0) {     //volume
            pc.printf("here#0: %d", acc_vol);
            if (vr1_ad > 0.1) {
                Direct_R = 0;
            } else {
                // nop;
            }

            if (vr1_ad < -0.1) {
                Direct_R = 1;
            } else {
                // nop;
            }
        } else {
            // nop;
        }
#endif
        if (acc_vol == 1) {
            Direct_R = (Direction.read() == 1) ? 0 : 1;
            pc.printf("here acc_vol: %d, Direct_R: %d, Direction: %d\r\n", acc_vol, Direct_R, Direction.read());
        } else {
            // nop;
        }
        /*
         * モードの更新
         */
        UVW = UVW_in(); /* センサ付・ホールIC信号 */
#if 1   
        if (Direct_R == 1) {
            switch (UVW) {
            case 1: PWM_Drive_M = 5;  break;
            case 2: PWM_Drive_M = 3;  break;
            case 3: PWM_Drive_M = 4;  break;
            case 4: PWM_Drive_M = 1;  break;
            case 5: PWM_Drive_M = 6;  break;
            case 6: PWM_Drive_M = 2;  break;
            default: PWM_Drive_M = 1; break;
            }
        } else {
            // nop;
        }
#endif
        if (Direct_R == 0) {
            switch (UVW) {
            case 1: PWM_Drive_M = 2;  break;
            case 2: PWM_Drive_M = 4;  break;
            case 3: PWM_Drive_M = 3;  break;
            case 4: PWM_Drive_M = 6;  break;
            case 5: PWM_Drive_M = 1;  break;
            case 6: PWM_Drive_M = 5;  break;
            default: PWM_Drive_M = 1; break;
            } 
        } else {
            // nop;
        }  
        if (rt_pwm == 0) {
#if 1  
            if (Direct_R == 0) {
                /************* PWM 駆動 0**** */
                switch (PWM_Drive_M) {
                case 1: 
                        PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0); 
                        UL=1; VL = 0; WL = 1;
                        break;
                case 2:
                        PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0);
                        UL=1; VL = 1; WL = 0;
                        break;
                case 3:
                        PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0);
                        UL = 1; VL = 1; WL = 0;
                        break;
                case 4:
                        PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0);
                        UL = 0; VL = 1; WL = 1;
                        break;
                case 5:
                        PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty);
                        UL = 0; VL = 1; WL = 1;
                        break;
                case 6:
                        PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty);
                        UL = 1; VL = 0; WL = 1;
                        break;
                default:
                        PWM_U.write(0); PWM_V.write(0); PWM_W.write(0);
                        UL = 1; VL = 1; WL = 1;
                        break; 
                }
            } else {
                // nop;
            }
#endif
#if 1  
            /*
             * PWM 駆動 1
             */            
            if (Direct_R == 1) {
                switch (PWM_Drive_M) {
                case 1:
                    PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0); 
                    UL = 1; VL = 1; WL= 0;
                    break;
                case 2:
                    PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0);
                    UL = 1; VL = 1; WL = 0;
                    break;
                case 3:
                    PWM_U.write(PWMDuty); PWM_V.write(0); PWM_W.write(0);
                    UL = 1; VL = 0; WL = 1;
                    break;
                case 4:
                    PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty);
                    UL = 1; VL = 0; WL = 1;
                    break;
                case 5:
                    PWM_U.write(0); PWM_V.write(0); PWM_W.write(PWMDuty);
                    UL = 0; VL = 1; WL = 1;
                    break;
                case 6:
                    PWM_U.write(0); PWM_V.write(PWMDuty); PWM_W.write(0);
                    UL = 0; VL = 1; WL = 1;
                    break;
                default:
                    PWM_U.write(0); PWM_V.write(0); PWM_W.write(0);
                    UL = 1; VL = 1; WL = 1;
                    break;
                }
            } else {
                // nop;
            }
#endif 
        } else {
            // rt_pwm != 0
        }

        // SWAVE = Nrpm / 10000;
        // SWAVE = Speed_diff / 500 + 0.5;
        // SWAVE = Speed_now / 750;
        // SWAVE = I_diff / 50 + 0.5;
        // SWAVE = vr1_ad;
        // SWAVE = PWMDuty;
        // SWAVE = Vpi;
        // SWAVE = curr_u;
        // SWAVE = I_diff / 24;
        // SWAVE = I_PI / 50 + 0.5;
        // SWAVE = I_PI / 100;
        // SWAVE = (I_detect - 0.5) * 303;
        // SWAVE = Speed_diff_norm + 0.5;
    }
}
