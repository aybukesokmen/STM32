

/*************************************
 * Versiyon: V 1.0.2
 * Tarih:
 * Yazan:
 * A��klama:  pwm ile steper motor s�r�lebildi.
 * 			  motor fren pinine 24V verilmelidir
 * 			  steepper_initialize() fonksiyonu olu�sturuldu. bu fonksiyon ba�lang��ta herdaim �a���r�lmal�d�r.
 *
 *
 ********************************/

/*21.10.21
 * STM32L053 ile step motor d�nd�r�ld�. Sa�a d�n-sola d�n komutlar� yaz�ld� fakat s�r�c�ye ba�land���nda �al��mad�.
 * 6400 de 8khz verildi.
 * ADC de�eri tan�mland� ve pota ba�land�. 8bit ile �al���ld�. Motorun sa�-sol olarak hareketi i�in kodlar yaz�ld�.
 * 1000 den1khz verildi.
 *
 ***********************************/
/* Versiyon: V 1.0.4
 * Tarih: 25.10.21
 * Yazan:
 * A��klama:1sn aral�klar ile timer interrupt olu�turuldu.
 * 			Sinyaller elde edildi.
 * 			  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim) fonksiyonu �a��r�ld�. Bu fonksiyon ilgili timer config ayar�n�n alt�na tan�mlanmal�d�r.
 * 			Sinyal elde edilmesi i�in SET-RESET komutlar� kullan�ld� ve dalga �retildi.

 *
 *
 *
/*
 *
 *
 *
 *
 * */
/*/* Versiyon: V 1.0.5
 * Tarih: 25.10.21
 * Yazan:Ayb�ke s
 * A��klama:2000 Hz ile �al���ld�.
 * 800 ad�m kullan�ld�
 * rpm 300
 * 1 tur 0.2 saniyede
 * pres 199,count 39 kullan�ld�
 *
 *void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	fonksyonu d�zenlendi.GL_timer_1ms_flag de�i�keni tan�mland�. Ve 0-1 de�erlerini ald�. 1ms lik timerlar olu�turuldu.
 *0 oldu�unda PA11 pini SET oldu,1 oldu�unda Reset oldu
 *
 *
 *
 *
 *
 ********************************************
 * * Versiyon: V 1.0.6
 * Tarih:
 * Yazan:
 * A��klama:1ms lik pulselar olu�tu. HAL_TIM_Base_Stop_IT(&htim2); fonksiyonu ile step motor istenen ad�mda durduruldu.
 * void stepper_pulse_kadar_dondur(unsigned  int istenen_pulse_sayisi) fonksiyonu olu�turuldu ve istenen pul ad�m� girildi�inde pulse say�m�z istenen pulsa
 * e�it olana kadar d�ng� devam eder. E��T �SE Timer stop edilir ve saya� de�erleri s�f�rlan�r . Bu �ekilde d�nd� devam eder
 *
 * ��Z�LEMEYENLER: Step ad�m ka��rmaktad�r.
 *
 *
 *
 *
 *
 *****************************************************
 *Versiyon: V 1.0.8
 *tAR�H:28.10.21
 *a�IKLAMA: 1600 Stepte 20HZ,200hz,2000hz de bak�ld� . STEP �al��maya ba�larken ad�m ka��r�yor.
 *1600 STEP 2000 hz h�zda �al���labilir
 *D�ng�y� HAL_TIM_PeriodElapsedCallback fonksiyonunda devam ettirdik.
 *Step motor s�r�c� CWD860-A ,sorun s�r�c�den kaynaklanabilir(?)
 *��Z�MLENEMEYENLER
 *Ba�lang��ta p�r�zs�z kalkmamas� ve sinyallerin bozuk gelmesi
 *
 *
 *
 *
 *
 * *Versiyon: V 1.0.9
 *tAR�H:1.11.21
 *A�IKLAMA: Potansiyometreyi devreye soktum. MAP fonksiyonu ile bit say�s�n� 35e b�ld�m. Amac�m pot ile frekans de�i�tirmekti. Fakat istedi�im sonucu alamad�m.
 *��Z�MLENEMEYENLER
 *ADC de�erinin okunmas�
 *
 *
 *
 *
 *
 * * *Versiyon: V 1.1.0
 *tAR�H:3.11.21
 *A�IKLAMA: 800,3200,25600 stepler ile denemeler yap�ld�. 8000hz ,1000hz verildi fakat ad�m ka��rmalar g�zlemlendi
 *rpm=150
 *Son De�i�kenler= 800 step,2000hz,75rpm ile �al���ld�. Enable aktif iken ad�m ka��rma g�zlemlendi.
 *Enable pasif duruma getirildi�inde ad�m ka��rmad�.
 *
 *
 *
 * * * *Versiyon: V 1.1.1
 *tAR�H:3.11.21 saat:14:23
 *A�IKLAMA: 800 step ,2khz ile �al���ld�.
 *motor_is_yap fonksiyonu yaz�ld�.Program ba�lad���nda ilk if in i�ine girdi,istenen tur say�s�na ula�ana kadar
 *elsenin i�ine girdi. ula�t���nda 2.else ifin i�ine girdi.
 *Tur say�s� ve bekleme s�resi main fonksiyonunun i�inde art�r�ld�.
 *istenen tur sayisi GLOBAL de�i�kene at�ld�.
 *Enable aktif de�il,tur ka��rma yok. UART Haberle�meye ge�ildi.
 *
 *
 *
 *
 ** * * *Versiyon: V 1.1.2
 *tAR�H:15.11.21 saat:17:18
 *A�IKLAMA: C# ile i�lemci kodlar� yaz�ld�. Her veri i�in fonksiyon olu�turuldu ve bu veriler ile de�i�kenler e�lendi.
 *PA5 ledi index numaras�na g�re yan�p s�nd�.
 *YAPILACAKLAR: c# kodlar� d�zenlenecek,WPF'ye aktar�lacak,step motorda kontrol� yap�lacak
 *
 * ** * * *Versiyon: V 1.1.3
 *tAR�H:23.11.21 saat:17:38
 *A�IKLAMA:sag_flag,sol_flag,tam_flag c#tan gelen verilere g�re kald�r�l�p indirildi. Motor d�nme say�s� ona g�re
 *if yap�s�n�n i�inde ayarland�.
 *��Z�MLENEMEYENLER:Fakat Transmitte sonradan bir s�k�nt� olu�tu.
 *
 *
 *
 *
 * * ** * * *Versiyon: V 1.1.4
 *tAR�H:25.11.2021
 *A�IKLAMA:HABERLE�ME �ALI�IYOR FAKAT BAZEN BOZUK PAKETLER GELEB�L�YOR. C# 'ta timer�n start edilmedi�i g�zlemlendi.
 *Timerda kodlar d�zenlendi FAKAT hala haberle�me sorunu devam ediyordu.Receive komutu while i�ine al�nd� ve sorunun ��z�ld��� g�zlemlendi.
 *
 *
 * * * ** * * *Versiyon: V 1.1.5
 *tAR�H:25.11.2021 saat:16:45
 *A�IKLAMA:array_kontrol_func fonksiyonu olu�turdu ve datan�n d�zg�n gelip gelmedi�ini �l�mek i�in c#tan +,- ve tam tur
 *butonlar�na bas�ld�. +da sa�a d�nmesi,-de sola d�nmesi sa�land�. sorunsuz �al��t�.
 *///////////////////////////////////////////////








