/**********************************************************/
/*Copyright ? 2021 Akiyoshi Oguro All Rights Reserved.*****/
/*矩形波駆動　CQBLDC_KIT+Nucleo_STM32F302R8+CQブラシレスモータ*/
/*EN1,EN2,EN3　→　UL、VL、WL 変更 20210331　*****************/
/*PWM優先駆動、PI制御周期化、アクセル⇔ボリューム切り替え機能追加**/
/*速度制御、電流制御、PWMDuty制御 切り替え機能追加***************/
/***************PWMDuty制御 切り替え機能変更****************/
/****************PI制御正規化*******************************/
/***************Ajust=0.7; //Nrpm_S回転速度計算値の調整******/
/*******************20210526*******************************/

#include "mbed.h"
#include <math.h>
#define VMAX_SET (1.0f) /* PI制御最大電圧指令0.0〜1.0 */
#define IMAX_SET (50.0f) /* PI制御最大電流指令0.0〜1.0 */

/**********PI制御周期設定*********************/
/*
 * コメントアウトでPI周期制御無し
 */
// #define TickSpeed
// #define TickCurrent

PwmOut PWM_U(PA_8);
PwmOut PWM_V(PA_9);
PwmOut PWM_W(PA_10);

DigitalOut UL(PA_7);
DigitalOut VL(PB_0);
DigitalOut WL(PB_1);

AnalogIn Curr_ui(PA_0);
AnalogIn Curr_vi(PC_1);
AnalogIn Curr_wi(PC_0);

// AnalogIn V_adc(PB_1);        // Blue On Board volume
// AnalogIn V_adc(PC_2);        // Gaibu Potention IHM07
// AnalogIn V_adc(PA_6);        // Gaibu Potention IHM08
AnalogIn V_adc(PC_2);           // Gaibu Potention CQInv_KIT

InterruptIn HA(PA_15);
InterruptIn HB(PB_3);
InterruptIn HC(PB_10);

DigitalIn U_in(PA_15);
DigitalIn V_in(PB_3);
DigitalIn W_in(PB_10);

DigitalIn Direction(PB_8);

Timer uTimer;
Timer vTimer;
Timer wTimer;

AnalogOut SWAVE(PA_4);

Serial pc(PC_10, PC_11);

/*************************************************************/
unsigned char acc_vol = 1;                  //オプション１ acc_vol==0 はVolume acc_vol==1はAccel
unsigned char Sp_tick = 0, Cu_tick = 0;     //周期PI制御フラグ
float Sp_ticktime = 160,
      Cu_ticktime = 80;                     //オプション２，３ PI制御周期時間、単位μsec
float change_pwm_N = 300;                    //オプション５ ホールセンサエッジPWM優先駆動開始回転数
unsigned char rt_pwm;                       // ホールセンサエッジ割り込みPWM優先駆動フラグ
                                            // rt_pwm＝１、優先無しrt_pwm＝0
/*************************************************************/

unsigned char HUVW;                         // サンプリング開始時のHole ICの状態

float iu_ad, iv_ad, iw_ad;
float Vu_ad, Vv_ad, Vw_ad;
float vr1_ad, vr_ad, vr1_ad_p;  // vr2_ad;
float Vr_adc_i;
float refu, refv, refw;
float curr_u, curr_v, curr_w;

float Nrpm = 0.0f;                             // 回転数[r/min]
float Nrpm_Previous = 0.0f;                    // 回転数[r/min] old
float Nrpm_s = 0;                           // 回転数[r/min]
float Speed_now = 0;
float Speed_MAX = 3500.0f;                   // 速度指令合大値 6直1500 3直2並2500 2直3並3500 6並6500
float Speed_SET = 0.0f;                      // 速度指令[r/min]
float Speed_SET0 = 0.0f;                     // 速度指令[r/min]
//
// つまみを回してからの目標値への伝達速度
//
float Accel_dN = 40.0f;                       // 加速レイト
float Decel_dN = 40.0f;                       // 減速レイト
float Speed_mini = 300.0f;
float Speed_err_MAX = 10000.0f;

/*
 * Hiroshima Pref. College modifications
 */
AnalogIn hCurrent(PA_2);            // Current value for Motor

struct hCurrentData
{
    float max;
    float min;
    float target;
    float current;
    float raw;
    float diff;
};

hCurrentData hcurr = {12000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

uint8_t UVW;
unsigned char U, V, W;
unsigned char PWM_Drive_M = 1;              // PWMモード
unsigned char Direct_R = 1;                 // Direct_R -> 1:正転，0:逆転

/*
 * 電流制御
 */
float kpCurrent = 0.000522f;                 // 比例ゲイン
float kiCurrent = 0.00022f;                  // 積分ゲイン
float s_kiCurrent = 0;
float I_PI = 0, I_PII = 1.0f;
float I_diff = 0;
float I_detect = 0;
float Vpi = 0;

/*
 * 速度制御
 */
float kaiten = 0;
float Speed_diff = 0.0f;                     // 速度誤差
//const float kpSpeed = 1.4f;                        // 比例ゲイン 0.4
const float kpSpeed = 5.0f;                        // 比例ゲイン 0.4
const float kiSpeed = 0.8f;                        // 積分ゲイン 0,2
float s_kiSpeed = 0.0f;                      // 積分器中身
float Speed_diff_norm = 0.0f;
float Ajust = 0.7f;                          // Nrpm_S回転速度計算値の調整

/*
 * Ticker
 */
Ticker Sp;
Ticker Cu;

/*******************************************/
float PWMDuty = 0;

/*
 * Hall sensors
 */
uint8_t UVW_in(void)
{
    return (uint8_t)((W_in << 2) | (V_in << 1) | U_in);
}

/*
 * Hall Caputure
 */
void Capture_u()
{
    static unsigned int Timer_cnt_A = 0;
    static unsigned int Timer_cnt_A_1 = 0;
    unsigned int t_cnt;
    float rpm = 0.0f;
    
    Timer_cnt_A = uTimer.read_us();
    t_cnt = Timer_cnt_A - Timer_cnt_A_1;
    if (t_cnt < 1) {
        t_cnt = 1;
    }

    rpm = 5000000.0f / t_cnt;
    Nrpm = Nrpm_Previous;
    if (fabs(rpm - Nrpm_Previous) < Speed_err_MAX) {
        Nrpm_Previous = rpm;
    }
    Timer_cnt_A_1 = Timer_cnt_A;
}

void Capture_v()
{
    static unsigned int Timer_cnt_B = 0;
    static unsigned int Timer_cnt_B_1 = 0;
    unsigned int t_cnt;
    float rpm = 0.0f;

    Timer_cnt_B = vTimer.read_us();

    t_cnt = Timer_cnt_B - Timer_cnt_B_1;
    if (t_cnt < 1) {
        t_cnt = 1;
    }

    rpm = 5000000.0f / t_cnt;
    Nrpm = Nrpm_Previous;
    if (fabs(rpm - Nrpm_Previous) < Speed_err_MAX) {
        Nrpm_Previous = rpm;
    }
    Timer_cnt_B_1 = Timer_cnt_B;
}

void Capture_w()
{
    static unsigned int Timer_cnt_C = 0;
    static unsigned int Timer_cnt_C_1 = 0;
    unsigned int t_cnt;
    float rpm = 0.0f;

    Timer_cnt_C = wTimer.read_us();

    t_cnt = Timer_cnt_C - Timer_cnt_C_1;
    if (t_cnt < 1) {
        t_cnt = 1;
    }

    rpm = 5000000.0f / t_cnt;
    Nrpm = Nrpm_Previous;
    if (fabs(rpm - Nrpm_Previous) < Speed_err_MAX) {
        Nrpm_Previous = rpm;
    }
    Timer_cnt_C_1 = Timer_cnt_C;
}

/*
 * Hall_u にキャプチャ発生
 */
void Hall_u()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(PWMDuty);
            PWM_V.write(0);
            PWM_W.write(0);
            UL = 1;
            VL = 0;
            WL = 1;  // 0:Active 1:High inpeedance
        }
        if (Direct_R == 1) {
            PWM_U.write(0);
            PWM_V.write(0);
            PWM_W.write(PWMDuty);
            UL = 1;
            VL = 0;
            WL = 1;  // 0:Active 1:High inpeedance
        }

    } else {
    }
    Capture_u();  //速度計算へ
}

void Hall_ul()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0);
            PWM_V.write(PWMDuty);
            PWM_W.write(0);
            UL = 0;
            VL = 1;
            WL = 1;  // 0:Active 1:High inpeedance
        }
        if (Direct_R == 1) {
            PWM_U.write(0);
            PWM_V.write(PWMDuty);
            PWM_W.write(0);
            UL = 1;
            VL = 1;
            WL = 0;  // 0:Active 1:High inpeedance
        }

    } else {
    }
    Capture_u();  //速度計算へ
}

/*
 * Hall_vにキャプチャ発生
 */
void Hall_v()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0);
            PWM_V.write(PWMDuty);
            PWM_W.write(0);
            UL = 1;
            VL = 1;
            WL = 0;  // 0:Active 1:High inpeedance
        }
        if (Direct_R == 1) {
            PWM_U.write(PWMDuty);
            PWM_V.write(0);
            PWM_W.write(0);
            UL = 1;
            VL = 1;
            WL = 0;  // 0:Active 1:High inpeedance
        }

    } else {
    }
    Capture_v();  //速度計算へ
}

void Hall_vl()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0);
            PWM_V.write(0);
            PWM_W.write(PWMDuty);
            UL = 1;
            VL = 0;
            WL = 1;  // 0:Active 1:High inpeedance
        }
        if (Direct_R == 1) {
            PWM_U.write(0);
            PWM_V.write(0);
            PWM_W.write(PWMDuty);
            UL = 0;
            VL = 1;
            WL = 1;  // 0:Active 1:High inpeedance
        }

    } else {
    }
    Capture_v();  //速度計算へ
}

/*
 * Hall_Wにキャプチャ発生
 */
void Hall_w()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(0);
            PWM_V.write(0);
            PWM_W.write(PWMDuty);
            UL = 0;
            VL = 1;
            WL = 1;  // 0:Active 1:High inpeedance
        }
        if (Direct_R == 1) {
            PWM_U.write(0);
            PWM_V.write(PWMDuty);
            PWM_W.write(0);
            UL = 0;
            VL = 1;
            WL = 1;  // 0:Active 1:High inpeedance
        }
    } else {
    }
    Capture_w();  //速度計算へ
}

void Hall_wl()
{
    if (rt_pwm == 1) {
        if (Direct_R == 0) {
            PWM_U.write(PWMDuty);
            PWM_V.write(0);
            PWM_W.write(0);
            UL = 1;
            VL = 1;
            WL = 0;  // 0:Active 1:High inpeedance
        }
        if (Direct_R == 1) {
            PWM_U.write(PWMDuty);
            PWM_V.write(0);
            PWM_W.write(0);
            UL = 1;
            VL = 0;
            WL = 1;  // 0:Active 1:High inpeedance
        }

    } else {
    }
    Capture_w();  //速度計算へ
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
        if (s_kiSpeed < (-IMAX_SET)) {
            s_kiSpeed = -IMAX_SET;
        } else {
        }
    }

    I_PI = s_kiSpeed + kpSpeed * Speed_diff;

    if (I_PI > IMAX_SET) {
        I_PI = IMAX_SET;
    } else {
        if (I_PI < (-IMAX_SET)) {
            I_PI = -IMAX_SET;
        } else {
        }
    }
}

void Current_PI()
{
    I_diff = (I_PI - (I_detect - 0.5f)) * I_PII;
    s_kiCurrent += kiCurrent * (I_diff);
    if (s_kiCurrent > VMAX_SET) {
        s_kiCurrent = VMAX_SET;
    } else {
        if (s_kiCurrent < (-VMAX_SET)) {
            s_kiCurrent = -VMAX_SET;
        } else {
        }
    }

    Vpi = s_kiCurrent + kpCurrent * I_diff;

    if (Vpi > VMAX_SET) {
        Vpi = VMAX_SET;
    } else {
        if (Vpi < (-VMAX_SET)) {
            Vpi = -VMAX_SET;
        } else {
        }
    }
}


/*
 * 電流測定関数 (平均と同じように変化)
 * 0.01 倍より 100 個の平均とほぼ同じ
 */
float h_calc_propotional(float val)
{
    static float pval = 0.0f;
    pval += (val - pval) * 0.005f;
    return pval;
}

/*
 * main 関数
 */
int main() 
{
    float volume_value = 0.0f;

    uTimer.start();
    vTimer.start();
    wTimer.start();

    PWM_U.period_us(25);
    PWM_V.period_us(25);
    PWM_W.period_us(25);

    HA.rise(&Hall_u);
    HA.fall(&Hall_ul);
    HB.rise(&Hall_v);
    HB.fall(&Hall_vl);
    HC.rise(&Hall_w);
    HC.fall(&Hall_wl);

    pc.baud(115200);

    wait_ms(500);
    Vr_adc_i = V_adc.read();

    pc.printf("*** Hello world ***\r");

#ifdef TickSpeed
    Sp.attach_us(&Speed_PI, Sp_ticktime);
    Sp_tick = 1;
#endif

#ifdef TickCurrent
    Cu.attach_us(&Current_PI, Cu_ticktime);
    Cu_tick = 1;
#endif

    while (1) {        
        vr_ad = V_adc.read();
        vr1_ad_p = (vr_ad - Vr_adc_i);          //ボリュームを使う場合
        // vr1_ad_p=(vr_ad-Vr_adc_i)*1.3;       //カート・キットのアクセルを使う場合
        vr1_ad += (vr1_ad_p - vr1_ad) * 0.2f;

//      pc.printf("* vr1_ad=%4.2f, PWMDuty=%4.2f, Vpi=%4.2f, raw=%.3f, cur=%6.0f, tar=%6.0f\r", 
//                   vr1_ad,       PWMDuty,       Vpi,       hcurr.raw, hcurr.current, hcurr.target);
        pc.printf("%4.1f,%4.1f,%3d\r", hcurr.target*0.001f, hcurr.current*0.001f, int(PWMDuty * 100));
#if 0
        if (fabs(vr1_ad) < 0.01f) {
            Speed_SET = 0.0f;
            Speed_SET0 = 0.0f;
            Speed_now = 0;
            I_PI = 0;
            I_diff = 0;
            Speed_diff = 0;
            Nrpm_s = 0;
            Nrpm = 0;
            PWMDuty = 0;

            s_kiCurrent = 0;
            s_kiSpeed = 0;
            I_PI = 0;
            I_diff = 0;
            I_detect = 0;
            Vpi = 0;
            kaiten = 0;
            rt_pwm = 0;
        }
#endif
        curr_u += (Curr_ui.read() - curr_u) * 0.1f;
        curr_v += (Curr_vi.read() - curr_v) * 0.1f;
        curr_w += (Curr_wi.read() - curr_w) * 0.1f;

        /*
         * 現在の回転速度 Nrpm を計算
         */
        Nrpm_s = Nrpm;

        /*
         * 現在のセンサー電流値測定
         */
        hcurr.raw = hCurrent.read();
        hcurr.current = h_calc_propotional( hcurr.raw ) * 17500.0f;

        /*
       　* ボリュームによる目標電流設定
         */    
        volume_value = fabs(hcurr.max * vr1_ad);
        if (volume_value > 1.0f) {
            if (hcurr.target > volume_value) {
                hcurr.target -= Decel_dN;
                if (hcurr.target < volume_value) {
                    hcurr.target = volume_value;
                }
            }
            if (hcurr.target < volume_value) {
                hcurr.target += Accel_dN;
                if (hcurr.target > volume_value) {
                    hcurr.target = volume_value;
                }
            }
//          if (hcurr.target < hcurr.min) {
//              hcurr.target = hcurr.min;
//          }
        }
        hcurr.diff = hcurr.target - hcurr.current;

#ifndef TickSpeed
        /*
         *　電流 PI 制御
         */
        s_kiSpeed += kiSpeed * hcurr.diff;
        if (s_kiSpeed > IMAX_SET) {
            s_kiSpeed = IMAX_SET;
        } else if (s_kiSpeed < (-IMAX_SET)) {
            s_kiSpeed = -IMAX_SET;
        }
        // 比例制御
        I_PI = s_kiSpeed + kpSpeed * hcurr.diff;
        if (I_PI > IMAX_SET) {
            I_PI = IMAX_SET;
        } else if (I_PI < (-IMAX_SET)) {
            I_PI = -IMAX_SET;
        }
#endif
        /* *********************************************** */
        /* [ 3 ] 電流制御 */
        /* *********************************************** */
        if (Direct_R == 0) {
            switch (PWM_Drive_M) {
                case 1:
                    I_detect = (curr_v);
                    break;
                case 2:
                    I_detect = (curr_w);
                    break;
                case 3:
                    I_detect = (curr_w);
                    break;
                case 4:
                    I_detect = (curr_u);
                    break;
                case 5:
                    I_detect = (curr_u);
                    break;
                case 6:
                    I_detect = (curr_v);
                    break;
                default:
                    I_detect = 0.0f;
            }
        }

        if (Direct_R == 1) {
            switch (PWM_Drive_M) {
                case 1:
                    I_detect = (curr_w);
                    break;
                case 2:
                    I_detect = (curr_w);
                    break;
                case 3:
                    I_detect = (curr_v);
                    break;
                case 4:
                    I_detect = (curr_v);
                    break;
                case 5:
                    I_detect = (curr_u);
                    break;
                case 6:
                    I_detect = (curr_u);
                    break;
                default:
                    I_detect = 0.0f;
            }
        }

#ifndef TickCurrent
        /*
         * 各モーター層への電流制御
         */
        I_diff = (I_PI - (I_detect - 0.5f)) * I_PII;
        s_kiCurrent += kiCurrent * I_diff;
        if (s_kiCurrent > VMAX_SET) {
            s_kiCurrent = VMAX_SET;
        } else if (s_kiCurrent < (-VMAX_SET)) {
            s_kiCurrent = -VMAX_SET;
        }

        Vpi = s_kiCurrent + kpCurrent * I_diff;

        if (Vpi > VMAX_SET) {
            Vpi = VMAX_SET;
        } else if (Vpi < (-VMAX_SET)) {
            Vpi = -VMAX_SET;
        }
#endif

        kaiten = fabs(Nrpm_s);
        // オプション 4
        // I_PII = 1.0f;
        // I_PII=fabs(vr1_ad);
        // 1. ボリュームが 0.01 以下に絞られている -> 0
        // 2. Vpi が 0.3 を下回っている -> 0.3
        // 3. それ以外 -> 計算された Vpi を Duty にする
        PWMDuty = (vr1_ad < 0.01f) ? 0.0f : (Vpi < 0.3f) ? 0.3f : Vpi;
        //pc.printf("kaiten:%f\r", kaiten);
        if (kaiten < change_pwm_N) {
            rt_pwm = 0;
            // rt_pwm = 1;
        } else {
            rt_pwm = 1;
        }

        if (acc_vol == 0) {  // volume
            if (vr1_ad > 0.1f) {
                Direct_R = 0;

            } else {
            }

            if (vr1_ad < -0.1f) {
                Direct_R = 1;

            } else {
            }
        } else {
        }

        if (acc_vol == 1) {  // Accel
            if (Direction == 1) {
                Direct_R = 0;
            } else {
            }

            if (Direction == 0) {
                Direct_R = 1;
            } else {
            }
        } else {
        }

        /*
         * モードの更新
         */
        UVW = UVW_in();     // センサ付・ホールIC信号

#if 1
        //pc.printf("UVW=%d,kaiten=%f\r\n", UVW, kaiten);
        if (Direct_R == 1) {
            switch (UVW) {
                case 1:
                    PWM_Drive_M = 5;
                    break;
                case 2:
                    PWM_Drive_M = 3;
                    break;
                case 3:
                    PWM_Drive_M = 4;
                    break;
                case 4:
                    PWM_Drive_M = 1;
                    break;
                case 5:
                    PWM_Drive_M = 6;
                    break;
                case 6:
                    PWM_Drive_M = 2;
                    break;
                default:
                    PWM_Drive_M = 1;
                    break;
            }
        } else {
        }
#endif

        if (Direct_R == 0) {
            switch (UVW) {
                case 1:
                    PWM_Drive_M = 2;
                    break;
                case 2:
                    PWM_Drive_M = 4;
                    break;
                case 3:
                    PWM_Drive_M = 3;
                    break;
                case 4:
                    PWM_Drive_M = 6;
                    break;
                case 5:
                    PWM_Drive_M = 1;
                    break;
                case 6:
                    PWM_Drive_M = 5;
                    break;
                default:
                    PWM_Drive_M = 1;
                    break;
            }
        } else {
        }
        
        if (rt_pwm == 0) {
#if 1
            if (Direct_R == 0) {
                /************* PWM 駆動 0**** */
                switch (PWM_Drive_M) {
                    case 1:
                        PWM_U.write(PWMDuty);
                        PWM_V.write(0);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 0;
                        WL = 1;  // 0:Active 1:High inpeedance
                        break;
                    case 2:
                        PWM_U.write(PWMDuty);
                        PWM_V.write(0);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 1;
                        WL = 0;
                        break;
                    case 3:
                        PWM_U.write(0);
                        PWM_V.write(PWMDuty);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 1;
                        WL = 0;
                        break;
                    case 4:
                        PWM_U.write(0);
                        PWM_V.write(PWMDuty);
                        PWM_W.write(0);
                        UL = 0;
                        VL = 1;
                        WL = 1;
                        break;
                    case 5:
                        PWM_U.write(0);
                        PWM_V.write(0);
                        PWM_W.write(PWMDuty);
                        UL = 0;
                        VL = 1;
                        WL = 1;

                        break;
                    case 6:
                        PWM_U.write(0);
                        PWM_V.write(0);
                        PWM_W.write(PWMDuty);
                        UL = 1;
                        VL = 0;
                        WL = 1;
                        break;
                    default:
                        PWM_U.write(0);
                        PWM_V.write(0);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 1;
                        WL = 1;
                        break;
                }
            } else {
            }
#endif

#if 1
            if (Direct_R == 1) {
                /************* PWM 駆動 1**** */
                switch (PWM_Drive_M) {
                    case 1:
                        PWM_U.write(0);
                        PWM_V.write(PWMDuty);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 1;
                        WL = 0;  // 0:Active 1:High inpeedance
                        break;
                    case 2:
                        PWM_U.write(PWMDuty);
                        PWM_V.write(0);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 1;
                        WL = 0;
                        break;
                    case 3:
                        PWM_U.write(PWMDuty);
                        PWM_V.write(0);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 0;
                        WL = 1;
                        break;
                    case 4:
                        PWM_U.write(0);
                        PWM_V.write(0);
                        PWM_W.write(PWMDuty);
                        UL = 1;
                        VL = 0;
                        WL = 1;
                        break;
                    case 5:
                        PWM_U.write(0);
                        PWM_V.write(0);
                        PWM_W.write(PWMDuty);
                        UL = 0;
                        VL = 1;
                        WL = 1;

                        break;
                    case 6:
                        PWM_U.write(0);
                        PWM_V.write(PWMDuty);
                        PWM_W.write(0);
                        UL = 0;
                        VL = 1;
                        WL = 1;
                        break;
                    default:
                        PWM_U.write(0);
                        PWM_V.write(0);
                        PWM_W.write(0);
                        UL = 1;
                        VL = 1;
                        WL = 1;
                        break;
                }
            } else {
            }

#endif
        } else {
        }  // rt_pwm==0

        // SWAVE=Nrpm/10000;
        // SWAVE=Speed_diff/500+0.5;
        // SWAVE=Speed_now/750;
        // SWAVE=I_diff/50+0.5;
        // SWAVE=vr1_ad;
        // SWAVE=PWMDuty;
        // SWAVE=Vpi;
        // SWAVE=curr_u;
        // SWAVE=I_diff/24;
        // SWAVE=I_PI/50+0.5;
        // SWAVE=I_PI/100;
        // SWAVE=(I_detect-0.5)*303;
        // SWAVE=Speed_diff_norm+0.5;
    }  // while
}  // main
