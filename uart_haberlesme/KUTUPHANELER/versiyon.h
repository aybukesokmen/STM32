/*
 * * Versiyon: V 1.0.0
 * Tarih:8.11.21
 * Saat:11.36
 * Yazan:AYB�KE
 * A��klama: Uart ile veri g�nderme i�lemi ger�ekle�tirildi. Realtermde g�nderilen karakterler g�zlemlendi.
 * FAKAT bazen yanl�� veri gidiyor.
 * baudrate:115200
 *
 *
 * /***********************************************************
 * * Versiyon: V 1.0.1
 * Tarih:8.11.21
 * Saat:17.26
 * Yazan:AYB�KE
 * A��klama:Veri alma g�nderme tamamland�.
 * ��z�lemeyenler:Baudrate h�z� 115600 fakat realtermde 57600 de �al���yor.
 ******************************************************************
 * * * Versiyon: V 1.0.2
 * Tarih:8.11.21
 * Saat:16:18
 * Yazan:AYB�KE
 * A��klama:Veri alma g�nderme tamamland�.
 * ��z�lemeyenler:3.veri g�nderiminde 1 ve 2. veriyi bellekte tutuyor .3.veriyi okumuyor.
 *
 *
 * * * * Versiyon: V 1.0.3
 * Tarih:10.11.21
 * Saat:15:36
 * Yazan:AYB�KE
 * A��klama:bir for d�ng�s� olu�turuldu. CRLF g�rd��� anda ge�mi� elemanlar�n yerine " " ekliyor. Bir nevi silinmi� oluyor.
 * ��z�lemeyenler:4*4 ten fazla veri yollanm�yor
 *
 *
 *  * * * * Versiyon: V 1.0.4
 * Tarih:11.11.21
 * Saat:17:41
 * Yazan:AYB�KE
 * A��klama:2.komutu g�nderebilmek i�in i�lemciyi resetlemem gerekiyordu. Bu sorun ��z�ld� ve Receive komutunu callback fonksiyonuna yazd�k
 * bu sayede her yazd�ktan sonra callback e girip yeni verileri yazd�rabiliyor.
 *
 *
 * *  * * * * Versiyon: V 1.0.5
 * Tarih:15.11.21
 * Saat:09:57
 * Yazan:AYB�KE
 * A��klama:C# tan gelen veriler i�in fonksiyonlar olu�turuldu. Her data geldi�inde index_no suna g�re user_led yanmaktad�r.
 * Step kodlar� ile birle�tirilecek. sorunsuz �al���yor.
 ********************************
 ********************************/
