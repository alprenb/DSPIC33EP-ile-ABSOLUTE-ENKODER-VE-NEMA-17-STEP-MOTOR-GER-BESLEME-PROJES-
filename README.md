# DSPIC33EP-ile-ABSOLUTE-ENKODER-VE-NEMA-17-STEP-MOTOR-GER-BESLEME-PROJES-
DSPIC33EP Mikroişlemcisini kullanarak enkoderden alınan geri besleme açı değerine göre step motorun hedef açıya ulaşan  geri beslemeli kapalı çevrim bir projedir.

__Temel Özellikler__
**Kapalı Çevrim Kontrol**: 16-bit SSI mutlak enkoder ile motor milinin anlık pozisyonu okunarak hedef açıya  yüksek doğrulukla ulaşılması sağlanır.
**Yumuşak Kalkış (Soft-Start Ramp)**: Timer1 kesmesi kullanılarak step motorun anlık yüksek frekanslarda kilitlenmesini önleyen hızlanma ivmesi (rampa) uygulanmıştır.
**Donanımsal Microstepping**: TB6600 sürücüsü üzerinden 1/8 mikroadımlama (M1=H, M2=L, M3=H) yapılandırılarak titreşimsiz ve sessiz dönüş sağlanır.
**Dinamik Akım Kontrolü (I2C)**: Motor torkunu ve sürücü akımını optimize etmek için I2C protokolü ile haberleşen MCP4725 DAC entegresi kullanılmıştır.
**Gerçek Zamanlı Telemetri**: Hedef açı, mevcut açı, hata payı ve motor çalışma durumu UART üzerinden 9600 baud rate ile PC'ye (veya kontrolcüye) aktarılır.
**Koruma Senaryoları**: LATCH, RESET, ENABLE, ve TQ (Tork) pinleri ile endüstriyel sürücü koruma mekanizmaları yönetilir.


**Mikrodenetleyici: Microchip dsPIC33EP512MU814**
**Motor Sürücü: TB6600 (veya benzeri endüstriyel step sürücü)**
**Motor: NEMA serisi Bipolar Step Motor**
**Sensör: 16-bit SSI Mutlak Enkoder**
**Ek Donanım: MCP4725 I2C DAC modülü**


__<img width="801" height="750" alt="MOTOR SÜRÜCÜ ŞEMA" src="https://github.com/user-attachments/assets/c6fa9399-7126-4665-99b1-5ed27b2812ab" />

Yazılım Mimarisi__
**Ana Döngü (main): SSI enkoderden raw (ham) veri okunur ve dereceye çevrilir. Hedef açı ile mevcut açı arasındaki fark (hata) hesaplanır. Hata belirlenen toleransın üzerindeyse yön belirlenir, sürücü uyandırılır (TQ_PIN = 0) ve motor_running bayrağı aktif edilir.**
**Timer1 Kesmesi (_T1Interrupt): motor_running bayrağı 1 olduğu sürece CLK pinini (XOR maskelemesi ile) tersler. Bu yapı, ana döngüyü bloklamadan (non-blocking) arka planda kararlı bir pulse üretilmesini sağlar.**
**SSI Haberleşmesi: Read_SSI_Encoder fonksiyonu, bit-banging yöntemi ile saat sinyali üretir ve veriyi senkron olarak okur. Zamanlamalar, enkoderin monoflop süresine (30µs) uygun olarak ayarlanmıştır.**
**I2C & DAC Yönetimi: Set_Motor_Current fonksiyonu ile sürücüye sağlanacak referans voltaj ayarlanır. (MCP4725 adres pini donanımsal olarak RD11 üzerinden toprağa çekilmiştir)**
