/*
 * * Versiyon: V 1.0.0
 * Tarih:8.11.21
 * Saat:11.36
 * Yazan:AYBÜKE
 * Açýklama: Uart ile veri gönderme iþlemi gerçekleþtirildi. Realtermde gönderilen karakterler gözlemlendi.
 * FAKAT bazen yanlýþ veri gidiyor.
 * baudrate:115200
 *
 *
 * /***********************************************************
 * * Versiyon: V 1.0.1
 * Tarih:8.11.21
 * Saat:17.26
 * Yazan:AYBÜKE
 * Açýklama:Veri alma gönderme tamamlandý.
 * Çözülemeyenler:Baudrate hýzý 115600 fakat realtermde 57600 de çalýþýyor.
 ******************************************************************
 * * * Versiyon: V 1.0.2
 * Tarih:8.11.21
 * Saat:16:18
 * Yazan:AYBÜKE
 * Açýklama:Veri alma gönderme tamamlandý.
 * Çözülemeyenler:3.veri gönderiminde 1 ve 2. veriyi bellekte tutuyor .3.veriyi okumuyor.
 *
 *
 * * * * Versiyon: V 1.0.3
 * Tarih:10.11.21
 * Saat:15:36
 * Yazan:AYBÜKE
 * Açýklama:bir for döngüsü oluþturuldu. CRLF gördüðü anda geçmiþ elemanlarýn yerine " " ekliyor. Bir nevi silinmiþ oluyor.
 * Çözülemeyenler:4*4 ten fazla veri yollanmýyor
 *
 *
 *  * * * * Versiyon: V 1.0.4
 * Tarih:11.11.21
 * Saat:17:41
 * Yazan:AYBÜKE
 * Açýklama:2.komutu gönderebilmek için iþlemciyi resetlemem gerekiyordu. Bu sorun çözüldü ve Receive komutunu callback fonksiyonuna yazdýk
 * bu sayede her yazdýktan sonra callback e girip yeni verileri yazdýrabiliyor.
 *
 *
 * *  * * * * Versiyon: V 1.0.5
 * Tarih:15.11.21
 * Saat:09:57
 * Yazan:AYBÜKE
 * Açýklama:C# tan gelen veriler için fonksiyonlar oluþturuldu. Her data geldiðinde index_no suna göre user_led yanmaktadýr.
 * Step kodlarý ile birleþtirilecek. sorunsuz çalýþýyor.
 ********************************
 ********************************/
