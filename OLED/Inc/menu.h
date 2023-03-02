/*
 * menu.h
 *
 *  Created on: Dec 5, 2022
 *      Author: aybuke.sokmen
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_


int count=0;



//HANGI BIT 1?

int	gl_strlen(char *str)
{
	int	menu_size;

	menu_size = 0;
	while (str[menu_size] != '\0')
	{
		menu_size++;
	}
	return (menu_size);
}


int binary_source(uint16_t adress)
{
	int min=1;
	min=min<<count;
	if(adress==min)
	{
		return count;
	}
	else
	{
		count++;
		binary_source(adress);

	}

}

uint16_t adres_bit=0x1000;
int gl_choice_flag=0;
//BUTON ADRESLERI
int menu_choice_func()
{
	int	n = binary_source(adres_bit);
	if(HAL_GPIO_ReadPin(GPIOB,adres_bit))
	 {
		gl_choice_flag=n;
		return n;
	 }
	else
	{
		adres_bit=adres_bit<<1;
		menu_choice_func();
	}

}

enum first_menu{
	A,
	B,
	C,
	D,
	E
};


//Menu header
int menu_size_func(int last_element)
{
	int menu_size;
	for(menu_size = 0; menu_size < last_element ; menu_size++)
	{
		gl_flag=menu_size;
	}
	return menu_size;

}



#endif /* INC_MENU_H_ */
