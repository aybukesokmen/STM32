/*
 * button.h
 *
 *  Created on: Dec 5, 2022
 *      Author: aybuke.sokmen
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_
#include "stdlib.h"

enum boolean {false, true};
int	count = 0;


enum gl_btn_flag
{gl_set_flag
	, gl_esc_flag,
	gl_up_flag,
	gl_down_flag
}gl_flag;

gl_flag=0;

int button_flag = 0;
int error_code=404;

//Butona basılıp basılmadıgını kontrol eder

/*
 * 1-button_choice_control fonksiyonu çağırılır
 * 2-Seçilen butonlar üstte tanımlanır
 * 3-button_choice_function çağırılır ve parametre olarak enum gl_btn_flag içindeki son eleman yazılır.
 * 4-Eger aşağıya ve yukarıya doğru saydırıp üst limitten sonra geri dönmesini istiyorsak ve alt limitten sonra son sayıya dönmesini istiyorsak
 * button_up_down_func(int menu_size) fonksiyonunu çağır.
 * */
int button_choice_control(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin))
	{
		button_flag=1;

	}
	else
	{
		button_flag=0;
	}
	return button_flag;
}



int button_choice_function(int btn_choice_flag)
{
	if(button_flag)
	{
		gl_flag=btn_choice_flag;
		return gl_flag;
	}
	else
	{
		return error_code;
	}

	/*
	 *Selection values ​​for  btn_choice_flag
	 * SET -> 0
	 * ESC -> 1
	 * UP  -> 2
	 * DOWN-> 3
	 * */
}

int button_up_down_func(int menu_size)
{
	if(button_flag)
	{
		if(gl_flag == gl_up_flag)
		{
			count ++;
		}
		else if (gl_flag == gl_down_flag)
		{
			count --;
		}

		if(count>= menu_size)
		{
			count = 0;
		}
		else if(count <0)
		{
			count = menu_size;
		}

	return count;
	}
	else
	{
		return error_code;
	}

	/*Arttırma ve azaltma butonları. Alt ve üst sınırlara göre başa döner.
	 *Sayac ile butonun konumu belirlenir.
	 */
}


#endif /* INC_BUTTON_H_ */
