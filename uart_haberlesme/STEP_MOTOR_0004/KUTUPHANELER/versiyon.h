

/*************************************
 * Versiyon: V 1.0.2
 * Tarih:
 * Yazan:
 * Açýklama:  pwm ile steper motor sürülebildi.
 * 			  motor fren pinine 24V verilmelidir
 * 			  steepper_initialize() fonksiyonu oluþsturuldu. bu fonksiyon baþlangýçta herdaim çaðýýrýlmalýdýr.
 *
 *
 ********************************/

/*21.10.21
 * STM32L053 ile step motor döndürüldü. Saða dön-sola dön komutlarý yazýldý fakat sürücüye baðlandýðýnda çalýþmadý.
 * 6400 de 8khz verildi.
 * ADC deðeri tanýmlandý ve pota baðlandý. 8bit ile çalýþýldý. Motorun sað-sol olarak hareketi için kodlar yazýldý.
 * 1000 den1khz verildi.
 *
 ***********************************/
/* Versiyon: V 1.0.4
 * Tarih: 25.10.21
 * Yazan:
 * Açýklama:1sn aralýklar ile timer interrupt oluþturuldu.
 * 			Sinyaller elde edildi.
 * 			  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim) fonksiyonu çaðýrýldý. Bu fonksiyon ilgili timer config ayarýnýn altýna tanýmlanmalýdýr.
 * 			Sinyal elde edilmesi için SET-RESET komutlarý kullanýldý ve dalga üretildi.

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
 * Yazan:Aybüke s
 * Açýklama:2000 Hz ile çalýþýldý.
 * 800 adým kullanýldý
 * rpm 300
 * 1 tur 0.2 saniyede
 * pres 199,count 39 kullanýldý
 *
 *void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	fonksyonu düzenlendi.GL_timer_1ms_flag deðiþkeni tanýmlandý. Ve 0-1 deðerlerini aldý. 1ms lik timerlar oluþturuldu.
 *0 olduðunda PA11 pini SET oldu,1 olduðunda Reset oldu
 *
 *
 *
 *
 *
 ********************************************
 * * Versiyon: V 1.0.6
 * Tarih:
 * Yazan:
 * Açýklama:1ms lik pulselar oluþtu. HAL_TIM_Base_Stop_IT(&htim2); fonksiyonu ile step motor istenen adýmda durduruldu.
 * void stepper_pulse_kadar_dondur(unsigned  int istenen_pulse_sayisi) fonksiyonu oluþturuldu ve istenen pul adýmý girildiðinde pulse sayýmýz istenen pulsa
 * eþit olana kadar döngü devam eder. EÞÝT ÝSE Timer stop edilir ve sayaç deðerleri sýfýrlanýr . Bu þekilde döndü devam eder
 *
 * ÇÖZÜLEMEYENLER: Step adým kaçýrmaktadýr.
 *
 *
 *
 *
 *
 *****************************************************
 *Versiyon: V 1.0.8
 *tARÝH:28.10.21
 *aÇIKLAMA: 1600 Stepte 20HZ,200hz,2000hz de bakýldý . STEP çalýþmaya baþlarken adým kaçýrýyor.
 *1600 STEP 2000 hz hýzda çalýþýlabilir
 *Döngüyü HAL_TIM_PeriodElapsedCallback fonksiyonunda devam ettirdik.
 *Step motor sürücü CWD860-A ,sorun sürücüden kaynaklanabilir(?)
 *ÇÖZÜMLENEMEYENLER
 *Baþlangýçta pürüzsüz kalkmamasý ve sinyallerin bozuk gelmesi
 *
 *
 *
 *
 *
 * *Versiyon: V 1.0.9
 *tARÝH:1.11.21
 *AÇIKLAMA: Potansiyometreyi devreye soktum. MAP fonksiyonu ile bit sayýsýný 35e böldüm. Amacým pot ile frekans deðiþtirmekti. Fakat istediðim sonucu alamadým.
 *ÇÖZÜMLENEMEYENLER
 *ADC deðerinin okunmasý
 *
 *
 *
 *
 *
 * * *Versiyon: V 1.1.0
 *tARÝH:3.11.21
 *AÇIKLAMA: 800,3200,25600 stepler ile denemeler yapýldý. 8000hz ,1000hz verildi fakat adým kaçýrmalar gözlemlendi
 *rpm=150
 *Son Deðiþkenler= 800 step,2000hz,75rpm ile çalýþýldý. Enable aktif iken adým kaçýrma gözlemlendi.
 *Enable pasif duruma getirildiðinde adým kaçýrmadý.
 *
 *
 *
 * * * *Versiyon: V 1.1.1
 *tARÝH:3.11.21 saat:14:23
 *AÇIKLAMA: 800 step ,2khz ile çalýþýldý.
 *motor_is_yap fonksiyonu yazýldý.Program baþladýðýnda ilk if in içine girdi,istenen tur sayýsýna ulaþana kadar
 *elsenin içine girdi. ulaþtýðýnda 2.else ifin içine girdi.
 *Tur sayýsý ve bekleme süresi main fonksiyonunun içinde artýrýldý.
 *istenen tur sayisi GLOBAL deðiþkene atýldý.
 *Enable aktif deðil,tur kaçýrma yok. UART Haberleþmeye geçildi.
 *
 *
 *
 *
 ** * * *Versiyon: V 1.1.2
 *tARÝH:15.11.21 saat:17:18
 *AÇIKLAMA: C# ile iþlemci kodlarý yazýldý. Her veri için fonksiyon oluþturuldu ve bu veriler ile deðiþkenler eþlendi.
 *PA5 ledi index numarasýna göre yanýp söndü.
 *YAPILACAKLAR: c# kodlarý düzenlenecek,WPF'ye aktarýlacak,step motorda kontrolü yapýlacak
 *
 * ** * * *Versiyon: V 1.1.3
 *tARÝH:23.11.21 saat:17:38
 *AÇIKLAMA:sag_flag,sol_flag,tam_flag c#tan gelen verilere göre kaldýrýlýp indirildi. Motor dönme sayýsý ona göre
 *if yapýsýnýn içinde ayarlandý.
 *ÇÖZÜMLENEMEYENLER:Fakat Transmitte sonradan bir sýkýntý oluþtu.
 *
 *
 *
 *
 * * ** * * *Versiyon: V 1.1.4
 *tARÝH:25.11.2021
 *AÇIKLAMA:HABERLEÞME ÇALIÞIYOR FAKAT BAZEN BOZUK PAKETLER GELEBÝLÝYOR. C# 'ta timerýn start edilmediði gözlemlendi.
 *Timerda kodlar düzenlendi FAKAT hala haberleþme sorunu devam ediyordu.Receive komutu while içine alýndý ve sorunun çözüldüðü gözlemlendi.
 *
 *
 * * * ** * * *Versiyon: V 1.1.5
 *tARÝH:25.11.2021 saat:16:45
 *AÇIKLAMA:array_kontrol_func fonksiyonu oluþturdu ve datanýn düzgün gelip gelmediðini ölçmek için c#tan +,- ve tam tur
 *butonlarýna basýldý. +da saða dönmesi,-de sola dönmesi saðlandý. sorunsuz çalýþtý.
 *///////////////////////////////////////////////








