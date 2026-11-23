/** furuta_full — impulse kick swing-up + balance */
#include <stdint.h>
#define GPIO(b,off) (*((volatile uint32_t*)((uint32_t)(b)+(off))))
#define GPIO_CRL(b) GPIO(b,0x00)
#define GPIO_CRH(b) GPIO(b,0x04)
#define GPIO_IDR(b) GPIO(b,0x08)
#define GPIO_ODR(b) GPIO(b,0x0C)
#define GPIOA 0x40010800UL
#define GPIOB 0x40010C00UL
#define GPIOC 0x40011000UL
#define RCC_BASE 0x40021000UL
#define RCC_APB2ENR (*((volatile uint32_t*)(RCC_BASE+0x18)))
#define RCC_APB1ENR (*((volatile uint32_t*)(RCC_BASE+0x1C)))
#define TIMx(b,off) GPIO(b,off)
#define TIM_CNT(b)  TIMx(b,0x24)
#define TIM_PSC(b)  TIMx(b,0x28)
#define TIM_ARR(b)  TIMx(b,0x2C)
#define TIM_SMCR(b) TIMx(b,0x08)
#define TIM_CCMR1(b)TIMx(b,0x18)
#define TIM_CCER(b) TIMx(b,0x20)
#define TIM_CR1(b)  TIMx(b,0x00)
#define TIM_CCR1(b) TIMx(b,0x34)
#define TIM_DIER(b) TIMx(b,0x0C)
#define TIM_SR(b)   TIMx(b,0x10)
#define TIM2 0x40000000UL
#define TIM3 0x40000400UL
#define TIM4 0x40000800UL
#define NVIC_ISER0 (*((volatile uint32_t*)0xE000E100))
#define PI 3.14159265f
#define TPI 6.2831853f
#define ZERO 920
#define CPR 844.8f
#define MAX_TORQUE 0.074f
#define PWM_SCALE (260.0f/MAX_TORQUE)
#define MAX_PWM 999
#define BAL_D      6.0f
#define PEND_COM   0.151f
#define ARM_LENGTH 0.160f
#define J_EFF      (0.0030f + 0.016f*ARM_LENGTH*ARM_LENGTH)
#define ENTER_DEG  30.0f
#define EXIT_DEG   40.0f
#define KICK_FWD_T   0.185f
#define KICK_REV_T   0.10f
#define KICK_REV_PWM 500
#define COAST_T     8.0f
#define IDLE_T      5.0f    /* 5s of stillness before retry */
#define DT 0.00025f
#define NSAMP 2000
static volatile uint32_t g_tick;
static float g_th_prev,g_td_filt,g_td_d;
static int g_state;  /* 0=KICK, 1=COAST_WAIT, 2=BALANCE, 3=IDLE_WAIT */
static uint32_t g_phase_end_tick,g_coast_end_tick,g_idle_ticks;
static int16_t g_last_pwm;
static uint16_t g_raw[NSAMP];
static int16_t  g_enc[NSAMP],g_pwm[NSAMP];
static uint32_t g_cnt;
static volatile uint16_t g_last_valid_raw;
static float my_fabsf(float x){return x>=0?x:-x;}
static void dly(void){for(volatile int i=0;i<4;i++);}
#define SDA_LO GPIO_ODR(GPIOB)&=~(1<<10)
#define SDA_HI GPIO_ODR(GPIOB)|=(1<<10)
#define SCL_LO GPIO_ODR(GPIOB)&=~(1<<11)
#define SCL_HI GPIO_ODR(GPIOB)|=(1<<11)
#define SDA_RD ((GPIO_IDR(GPIOB)>>10)&1)
static void i2s(void){SDA_HI;SCL_HI;dly();SDA_LO;dly();SCL_LO;}
static void i2p(void){SDA_LO;dly();SCL_HI;dly();SDA_HI;dly();}
static int i2w(uint8_t b){for(int i=7;i>=0;i--){if(b&(1<<i))SDA_HI;else SDA_LO;dly();SCL_HI;dly();SCL_LO;}SDA_HI;dly();SCL_HI;dly();int a=SDA_RD==0;SCL_LO;return a;}
static uint8_t i2r(int ack){uint8_t b=0;SDA_HI;for(int i=7;i>=0;i--){SCL_HI;dly();if(SDA_RD)b|=(1<<i);SCL_LO;dly();}if(ack)SDA_LO;else SDA_HI;dly();SCL_HI;dly();SCL_LO;return b;}
static uint16_t as5600(void){i2s();i2w(0x6C);i2w(0x0E);i2s();i2w(0x6D);uint8_t hi=i2r(1),lo=i2r(0);i2p();return((uint16_t)hi<<8)|lo;}
static float wrap2pi(float r){while(r<0)r+=TPI;while(r>=TPI)r-=TPI;return r;}
static float wrap_pi(float r){while(r>PI)r-=TPI;while(r<-PI)r+=TPI;return r;}
static void motor(int16_t pwm){if(pwm>MAX_PWM)pwm=MAX_PWM;if(pwm<-MAX_PWM)pwm=-MAX_PWM;if(pwm==0){GPIO_ODR(GPIOA)&=~((1<<5)|(1<<6));TIM_CCR1(TIM2)=0;}else if(pwm>0){GPIO_ODR(GPIOA)=(GPIO_ODR(GPIOA)&~(1<<6))|(1<<5);TIM_CCR1(TIM2)=pwm;}else{GPIO_ODR(GPIOA)=(GPIO_ODR(GPIOA)&~(1<<5))|(1<<6);TIM_CCR1(TIM2)=-pwm;}}
void TIM3_IRQHandler(void){TIM_SR(TIM3)&=~1;g_tick++;}
void main(void){
    RCC_APB2ENR|=0x1C|(1<<0);RCC_APB1ENR|=(1<<0)|(1<<1)|(1<<2);
    GPIO_CRL(GPIOA)=(GPIO_CRL(GPIOA)&~(0xF<<0))|(0xB<<0);
    GPIO_CRL(GPIOA)=(GPIO_CRL(GPIOA)&~(0xFF<<20))|(0x22<<20);
    GPIO_CRL(GPIOA)=(GPIO_CRL(GPIOA)&~(0xF<<28))|(0x2<<28);
    GPIO_CRL(GPIOB)=(GPIO_CRL(GPIOB)&~(0xFF<<24))|(0x44<<24);
    GPIO_CRH(GPIOB)=(GPIO_CRH(GPIOB)&~(0xFF<<8))|(0x77<<8);GPIO_ODR(GPIOB)|=(3<<10);
    GPIO_CRH(GPIOC)=(GPIO_CRH(GPIOC)&~(0xF<<20))|(0x2<<20);GPIO_ODR(GPIOA)|=(1<<7);
    TIM_PSC(TIM2)=7;TIM_ARR(TIM2)=999;TIM_CCMR1(TIM2)=(6<<4);TIM_CCER(TIM2)=1;TIM_CR1(TIM2)=1;
    TIM_SMCR(TIM4)=3;TIM_CCMR1(TIM4)=1|(1<<8);TIM_ARR(TIM4)=0xFFFF;TIM_CR1(TIM4)=1;
    TIM_PSC(TIM3)=1;TIM_ARR(TIM3)=999;TIM_DIER(TIM3)=1;NVIC_ISER0=1<<29;TIM_CR1(TIM3)=1;
    GPIO_ODR(GPIOC)|=(1<<13);uint32_t lt=0;
    uint16_t raw=as5600();
    g_th_prev=wrap2pi((float)((int32_t)(raw-ZERO))*TPI/4096.f);
    g_td_filt=0;g_td_d=0;g_state=0;g_cnt=0;
    g_last_valid_raw=ZERO;
    g_phase_end_tick=0;
    g_coast_end_tick=0;
    g_idle_ticks=0;
    for(;;){uint32_t t=g_tick;if(t==lt)continue;lt=t;
        raw=as5600();
        if(raw>50 && raw<4050){g_last_valid_raw=raw;}else{raw=g_last_valid_raw;}
        float th=wrap2pi((float)((int32_t)(raw-ZERO))*TPI/4096.f);
        float d=th-g_th_prev;if(d>PI)d-=TPI;else if(d<-PI)d+=TPI;
        g_td_filt=0.5f*g_td_filt+0.5f*(d/DT);
        g_td_d=0.13f*g_td_d+0.87f*(d/DT);
        g_th_prev=th;
        float phi=wrap_pi(th-PI);float td=g_td_filt;float phi_deg=my_fabsf(phi)*57.29578f;
        int16_t pwm=0;
        /* balance always has top priority */
        if(g_state!=2 && phi_deg<ENTER_DEG && raw>100 && raw<4000) g_state=2;
        if(g_state==2 && phi_deg>EXIT_DEG) g_state=3;
        if(g_state==2 && raw>50 && raw<4050){
            float p_mag=0;
            if(phi_deg>0.5f){
                if(phi_deg>=15.0f)p_mag=999;
                else p_mag=80+(phi_deg-0.5f)/(15.0f-0.5f)*(999-80);
            }
            float p_pwm=(phi>=0)?-p_mag:p_mag;
            if(phi_deg<15.0f) g_last_pwm+=(int32_t)(J_EFF*(PEND_COM/ARM_LENGTH)*(-BAL_D*g_td_d)*PWM_SCALE);
            /* P is instant, D is averaged over 20ms */
            int16_t d_avg = (int16_t)(g_last_pwm/8);
            pwm = (int16_t)(p_pwm + d_avg);
            if(pwm>999)pwm=999;if(pwm<-999)pwm=-999;
            if((t%8)==7) g_last_pwm=0;
        }
        /* then the state machine for swing-up/idle */
        if(g_state==0){
            /* KICK: 0.17s fwd 100% → 0.10s rev 50% → coast */
            uint32_t dt_abs = t - g_phase_end_tick;  /* time since kick start */
            uint32_t rev_start=(uint32_t)(KICK_FWD_T/DT);
            uint32_t rev_end=rev_start+(uint32_t)(KICK_REV_T/DT);
            if(dt_abs<rev_start){if(!pwm)pwm=999;}
            else if(dt_abs<rev_end){if(!pwm)pwm=-KICK_REV_PWM;}
            else{g_state=3;}
        }else{
            /* IDLE_WAIT */
            if(my_fabsf(td)<1.5f && phi_deg>EXIT_DEG){g_idle_ticks++;}else{g_idle_ticks=0;}
            if(g_idle_ticks>(uint32_t)(IDLE_T/DT)){g_state=0;g_idle_ticks=0;g_phase_end_tick=t;}
        }
        if(phi_deg<ENTER_DEG)GPIO_ODR(GPIOC)&=~(1<<13);else GPIO_ODR(GPIOC)|=(1<<13);
        motor(pwm);
        if(g_cnt<NSAMP&&(t%20)==0){g_raw[g_cnt]=raw;g_enc[g_cnt]=0;g_pwm[g_cnt]=pwm;g_cnt++;}
    }
}
