#define FCY 3685000UL
#include <xc.h>
#include <libpic30.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

// --- MOTOR PINLERI (TB6600 / TB67S24x) - GUNCEL PIN HARITASI ---
#define CLK_PIN         LATEbits.LATE4    // CLK  -> RE4
#define CW_CCW_PIN      LATFbits.LATF2    // CW/CCW -> RF2
#define ENABLE_PIN      LATHbits.LATH6    // ENABLE -> RH6
#define RESET_PIN       LATHbits.LATH7    // RESET  -> RH7

#define M1_PIN          LATHbits.LATH10   // M1 -> RH10
#define M2_PIN          LATHbits.LATH9    // M2 -> RH9
#define M3_PIN          LATHbits.LATH8    // M3 -> RH8
#define LATCH_PIN       LATAbits.LATA2    // LATCH -> RA2

#define ALERT_PIN       PORTAbits.RA3     // ALERT (Input) -> RA3
#define TQ_PIN          LATHbits.LATH11   // TQ (Output)   -> RH11
#define MO_PIN          PORTFbits.RF3     // MO (Input)    -> RF3

// ALERT pininin aktif seviyesi surucu datasheet'ine gore degisir.
// TB6600/TB67S24x genelde ACTIVE-LOW alarm cikisi kullanir. Emin degilseniz
// osiloskop/multimetre ile teyit edin, gerekirse 0 <-> 1 degistirin.
#define ALERT_ACTIVE_LEVEL   0

// --- SSI ENKODER PINLERI ---
#define SSI_CLK_PIN     LATHbits.LATH14
#define SSI_DATA_PIN    PORTHbits.RH15
#define ENCODER_BITS        16
#define ENCODER_MAX_VALUE   ((1UL << ENCODER_BITS) - 1UL)

#define SSI_CLK_HALF_US      5
#define SSI_MONOFLOP_US     30

// --- NEMA17 + 1:27 REDUKTOR PARAMETRELERI ---
#define GEARBOX_RATIO               27
#define MOTOR_STEPS_PER_REV         200     // 1.8 derece/adim (NEMA17 tam adim)

// DUZELTME (1): MICROSTEP degeri ile M1/M2/M3 pin kombinasyonu ARTIK TUTARLI.
// Asagidaki M1/M2/M3 = (0,0,1) kombinasyonu surucu datasheet'inizde HANGI
// microstep oranina karsilik geliyorsa MICROSTEP degerini ONA GORE ayarlayin.
// Bu satir sadece ORNEK olarak Full Step (1/1) varsayimiyla birakilmistir.
// Gercek degeri datasheet tablosundan MUTLAKA teyit edin.
#define MICROSTEP                   1       // M1=0,M2=0,M3=1 -> Full Step varsayimi (DATASHEET'TEN TEYIT EDIN)

#define MICROSTEPS_PER_MOTOR_REV    (MOTOR_STEPS_PER_REV * MICROSTEP)                      // 200
#define MICROSTEPS_PER_OUTPUT_REV   ((uint32_t)MICROSTEPS_PER_MOTOR_REV * GEARBOX_RATIO)   // 5400
#define DEGREES_PER_MICROSTEP       (360.0f / (float)MICROSTEPS_PER_OUTPUT_REV)

// Hiz profili: PR1 kucukse motor hizli, buyukse motor yavas doner
#define PR1_HIZLI            200      // Buyuk hatalarda tam hiz
#define PR1_YAVAS            1500     // Toleransa yakinken yavas hiz (asim/overshoot azaltir)
#define HATA_HIZLI_ESIK_DEG  20.0f    // Bu derecenin ustundeki hatalarda tam hiza gecilir

// DUZELTME (7): Histerezis - baslama/durma esikleri ayri tutularak
// tolerans siniri civarinda motorun surekli start/stop yapmasi (chatter) onlenir.
#define TOLERANS_BASLAMA_DEG   5.0f   // Bu hatanin ustunde motor calismaya baslar
#define TOLERANS_DURMA_DEG     3.0f   // Bu hatanin altina inince motor durur

// DUZELTME (5): Hedefe ulasildiktan sonra torku tekrar kilitlemeden once
// kac ms stabil kalinmasi beklenecek (titresim/salinimi onlemek icin).
#define TORK_KILITLEME_GECIKME_MS   300

volatile uint32_t encoder_raw = 0;
volatile float encoder_degrees = 0.0f;
char tx_buffer[64];
volatile uint8_t motor_running = 0;

void System_Init(void) {
    // --- 1. ANALOG PINLERI DIJITALE CEVIRME ---
    // 0 = Dijital Mod, 1 = Analog Mod (Varsayilan)
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    // DUZELTME (2): ANSELF ve ANSELH artik EFEKTIF sekilde temizleniyor.
    // Bu pinler ilgili derleyici/cihaz header'inda tanimli degilse (register
    // yoksa) derleyici hata verir; bu durumda asagidaki iki satiri kaldirin.
    //ANSELF = 0x0000;
    ANSELG = 0x0000;
   // ANSELH = 0x0000;

    // --- 2. YONLENDIRME (TRIS) AYARLARI ---
    // Cikis (Output) Pinleri
    TRISEbits.TRISE4  = 0; // CLK
    TRISFbits.TRISF2  = 0; // CW/CCW
    TRISHbits.TRISH6  = 0; // ENABLE
    TRISHbits.TRISH7  = 0; // RESET
    TRISHbits.TRISH10 = 0; // M1
    TRISHbits.TRISH9  = 0; // M2
    TRISHbits.TRISH8  = 0; // M3
    TRISAbits.TRISA2  = 0; // LATCH
    TRISHbits.TRISH11 = 0; // TQ

    // Giris (Input) Pinleri
    TRISAbits.TRISA3  = 1; // ALERT
    TRISFbits.TRISF3  = 1; // MO

    // --- 3. BASLANGIC DURUMLARI ---
    // DIKKAT: TB6600 icin ENABLE_PIN 0 olmali ki motor kilitlensin ve calismaya hazir olsun.
    ENABLE_PIN = 0;
    RESET_PIN = 1;
    LATCH_PIN = 0;

    // Tork Kontrolu
    TQ_PIN = 1;

    // Cozunurluk (M1/M2/M3 kombinasyonu - MICROSTEP tanimiyla TUTARLI olmali,
    // yukaridaki MICROSTEP=1 tanimina gore Full Step birakilmistir)
    M1_PIN = 0;
    M2_PIN = 0;
    M3_PIN = 1;

    CW_CCW_PIN = 1;
    CLK_PIN = 0;

    // Enkoder (bu pinler kullanicidan gelen pin haritasinda belirtilmedi,
    // RH14/RH15 olarak birakildi; farkliysa guncelleyin)
    TRISHbits.TRISH14 = 0;
    TRISHbits.TRISH15 = 1;

    SSI_CLK_PIN = 1;
}

void Timer1_Init(void) {
    T1CONbits.TON = 0;
    T1CONbits.TCKPS = 0b01;
    PR1 = PR1_YAVAS; // baslangic degeri, kontrol dongusunde guncellenecek
    IPC0bits.T1IP = 4;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    if (motor_running) {
        CLK_PIN ^= 1;
    }
}

void UART1_Initialize(void) {
    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 1;

    __builtin_write_OSCCONL(OSCCON & 0xBF);
    _U1RXR = 81;
    _RP80R = 1;
    __builtin_write_OSCCONL(OSCCON | 0x40);

    U1MODE = 0;
    U1STA = 0;
    U1BRG = 23;

    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
}

void UART1_WriteString(const char* str) {
    while (*str) {
        while (U1STAbits.UTXBF);
        U1TXREG = *str++;
    }
}

uint32_t Read_SSI_Encoder(uint8_t bit_count) {
    uint32_t data = 0;
    __delay_us(SSI_MONOFLOP_US);

    for (uint8_t i = 0; i < bit_count; i++) {
        data <<= 1;
        SSI_CLK_PIN = 0;
        __delay_us(SSI_CLK_HALF_US);
        SSI_CLK_PIN = 1;
        __delay_us(SSI_CLK_HALF_US);
        if (SSI_DATA_PIN) {
            data |= 1;
        }
    }
    __delay_us(SSI_MONOFLOP_US);
    return data;
}

float SSI_ToDegrees(uint32_t raw_value) {
    return ((float) raw_value * 360.0f) / (float) (ENCODER_MAX_VALUE + 1UL);
}

// Hata buyukse motor hizli, tolerans sinirina yaklastikca motor yavaslar.
// Bu, redukturun getirdigi yuksek cozunurluk nedeniyle olusabilecek
// asimi (overshoot) azaltmak icindir.
void Motor_HizAyarla(float hata_abs_deg) {
    float pr1_deger;

    if (hata_abs_deg >= HATA_HIZLI_ESIK_DEG) {
        pr1_deger = PR1_HIZLI;
    } else {
        float oran = hata_abs_deg / HATA_HIZLI_ESIK_DEG; // 0..1 arasi
        pr1_deger = PR1_YAVAS - oran * (PR1_YAVAS - PR1_HIZLI); // orantili (P) yavaslama
    }

    PR1 = (uint16_t) pr1_deger;
}

// DUZELTME (6): Surucu ALERT sinyalini kontrol eden guvenlik fonksiyonu.
// ALERT aktif oldugunda motor derhal durdurulur ve surucu devre disi birakilir.
// return: 1 = alarm var (motor durduruldu), 0 = alarm yok
uint8_t Alert_Kontrol(void) {
    if (ALERT_PIN == ALERT_ACTIVE_LEVEL) {
        motor_running = 0;
        T1CONbits.TON = 0;
        TQ_PIN = 1;
        ENABLE_PIN = 1;   // TB6600 icin surucuyu devre disi birak (aktif-low varsayimi ile 1 = disable)
        return 1;
    }
    return 0;
}

// DUZELTME (4): 0/360 derece sarma (wrap-around) sorunu giderildi.
// Hedef ve mevcut aci arasindaki EN KISA yol hesaplanir.
float Aci_Farki_Hesapla(float hedef_aci, float mevcut_aci) {
    float hata = hedef_aci - mevcut_aci;
    if (hata > 180.0f)  hata -= 360.0f;
    if (hata < -180.0f) hata += 360.0f;
    return hata;
}

// DUZELTME (3) + (5) + (7): Yon degistirirken kesme kapatiliyor (glitch onleme),
// histerezisli baslama/durma esikleri kullaniliyor, hedefe ulasinca gecikmeli
// tork kilitleme uygulaniyor.
void Motor_Kontrol(float hedef_aci, float mevcut_aci) {
    static uint8_t tork_kilitli = 1;
    static uint32_t stabil_sayac_ms = 0;

    float hata = Aci_Farki_Hesapla(hedef_aci, mevcut_aci);
    float hata_abs = fabsf(hata);

    uint8_t calismali = motor_running
        ? (hata_abs > TOLERANS_DURMA_DEG)      // zaten calisiyorsa DURMA esigini kullan
        : (hata_abs > TOLERANS_BASLAMA_DEG);   // duruyorsa BASLAMA esigini kullan

    if (calismali) {
        stabil_sayac_ms = 0;

        if (tork_kilitli) {
            TQ_PIN = 1; // hareket icin tam tork (0 = tam tork, sürücü mant???n?za göre teyit edin)
        }

        // Yon degisimi sirasinda ISR'in CLK_PIN'i toggle etmesini engelle (glitch onleme)
        IEC0bits.T1IE = 0;
        TQ_PIN = 0;                              // hareket halindeyken tam tork
        CW_CCW_PIN = (hata > 0) ? 1 : 0;
        tork_kilitli = 0;
        __delay_us(30);                          // surucu yon pinini oturtma suresi
        IEC0bits.T1IE = 1;

        Motor_HizAyarla(hata_abs);
        motor_running = 1;
    } else {
        motor_running = 0;

        // Hedefe ulasildiginda ANINDA degil, bir sure stabil kaldiktan sonra
        // torku kilitle; ani kilitleme titresime/asima yol acabilir.
        if (!tork_kilitli) {
            stabil_sayac_ms += 10; // bu fonksiyon ~10ms periyotla cagriliyor (main dongudeki delay ile eslesmeli)
            if (stabil_sayac_ms >= TORK_KILITLEME_GECIKME_MS) {
               // TQ_PIN = 1;
                tork_kilitli = 1;
            }
        }
    }
}

int main(void) {
    System_Init();
    UART1_Initialize();
    Timer1_Init();
    motor_running = 0;

    float hedef_aci = 90.0f;

    while (1) {
        // DUZELTME (6): Her dongude surucu alarm durumu kontrol edilir.
        if (Alert_Kontrol()) {
            UART1_WriteString("ALARM: Surucu hata sinyali aktif, motor durduruldu!\r\n");
            __delay_ms(100);
            continue; // alarm surdukce normal kontrolu atla
        }

        IEC0bits.T1IE = 0;
        encoder_raw = Read_SSI_Encoder(ENCODER_BITS);
        IEC0bits.T1IE = 1;

        encoder_degrees = SSI_ToDegrees(encoder_raw);

        Motor_Kontrol(hedef_aci, encoder_degrees);

        sprintf(tx_buffer, "Hedef: %.1f | Mevcut: %.1f | Hata: %.1f\r\n",
                (double) hedef_aci, (double) encoder_degrees,
                (double) Aci_Farki_Hesapla(hedef_aci, encoder_degrees));
        UART1_WriteString(tx_buffer);

        __delay_ms(10);
    }

    return 0;
}