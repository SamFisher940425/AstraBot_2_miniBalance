/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/

#ifndef __OLED_H
#define __OLED_H			  	 

#include "sys.h"



/* 定义OLED连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的OLED引脚 */
#define OLED_SCLK_GPIO_PORT    	GPIOC			              /* GPIO端口 */
#define OLED_SCLK_GPIO_CLK 	    RCC_APB2Periph_GPIOC		/* GPIO端口时钟 */
#define OLED_SCLK_GPIO_PIN		GPIO_Pin_14			        /* 相应引脚号 */

#define OLED_SDIN_GPIO_PORT    	GPIOB			              /* GPIO端口 */
#define OLED_SDIN_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define OLED_SDIN_GPIO_PIN		GPIO_Pin_5			        /* 相应引脚号 */

#define OLED_RST_GPIO_PORT    	GPIOB			              /* GPIO端口 */
#define OLED_RST_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define OLED_RST_GPIO_PIN		GPIO_Pin_4			        /* 相应引脚号 */

#define OLED_RS_GPIO_PORT    	GPIOB			              /* GPIO端口 */
#define OLED_RS_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define OLED_RS_GPIO_PIN		GPIO_Pin_3			        /* 相应引脚号 */




//-----------------OLED端口定义---------------- 
#define OLED_RST_Clr() PBout(4)=0   //RST
#define OLED_RST_Set() PBout(4)=1   //RST

#define OLED_RS_Clr() PBout(3)=0    //DC
#define OLED_RS_Set() PBout(3)=1    //DC

#define OLED_SCLK_Clr()  PCout(14)=0  //SCL
#define OLED_SCLK_Set()  PCout(14)=1   //SCL

#define OLED_SDIN_Clr()  PBout(5)=0   //SDA
#define OLED_SDIN_Set()  PBout(5)=1   //SDA

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据





extern u8 OLED_GRAM[128][8];	 

//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	
void OLED_Refresh_GRAM_Line(u8 line);

#endif  
	 
