#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

static int addr = 0x76;

struct bmp384_calib_param {  //Structure qui stocke les paramétres pour ajuster la pression et la temperature.

   uint16_t nvm_t1;
   uint16_t nvm_t2;
   int8_t nvm_t3;
   
   int16_t nvm_p1;
   int16_t nvm_p2;
   
   int8_t nvm_p3;
   int8_t nvm_p4;
   
   uint16_t nvm_p5;
   uint16_t nvm_p6;
   
   int8_t nvm_p7;
   int8_t nvm_p8;
   
   int16_t nvm_p9;
   
   int8_t nvm_p10;
   int8_t nvm_p11;
   
   float par_t1;
   float par_t2;
   float par_t3;
   
   float par_p1;
   float par_p2;
   
   float par_p3;
   float par_p4;
   
   float par_p5;
   float par_p6;
   
   float par_p7;
   float par_p8;
   
   float par_p9;
   
   float par_p10;
   float par_p11;
   
   float t_lin;
   
    
};

#ifdef i2c_default
static void bmp_init(){   // Fonction qui active les mesures de la pression et de la temperature.

 uint8_t buf[] = {0x1B, 0x33};
 i2c_write_blocking(i2c_default, addr, buf, 2, false); // On écrit dans le registe 0x1B la valeur 0011 0011

}




static void bmp_read (uint32_t* temp, uint32_t* press){    // Fonction qui lis la pression et la temperature.
 
 uint8_t buffer[6];
 uint8_t val = 0x04; 

 i2c_write_blocking(i2c_default, addr, &val, 1, true); // On appelle le registre qui a pour adresse 0x04 

 i2c_read_blocking(i2c_default, addr, buffer, 6, false); // On lis les trois octets qui représente la pression et les trois octets qui représente la temperature.
 
 *press = (buffer[2] << 16) | ( buffer[1] << 8) | buffer[0]; // On range les octets de poids fort et de poids faibles (MSB,LSB,XLSB)
 *temp = (buffer[5] << 16) | (buffer[4] << 8) | buffer[3];
  
 
}

static float bmp384_temp(uint32_t temp_uncomp, struct bmp384_calib_param* params){ // Cette fonction utilise les paramétres présent ci-dessus dans la struture pour ajuster la valeur de la température


       float partial_data1;
       float partial_data2;
       
       partial_data1 = (float)(temp_uncomp - params->par_t1);
       partial_data2 = (float)(partial_data1 * params->par_t2);
       
       params->t_lin = partial_data2 + (partial_data1 * partial_data1) * params->par_t3;
       
       
       return params->t_lin;

}


static float bmp384_press(uint32_t press_uncomp, struct bmp384_calib_param* params){  // Cette fonction utilise les paramétres présent ci-dessus dans la struture pour ajuster la pression

       float comp_press;
       
       float partial_data1;
       float partial_data2;
       float partial_data3;
       float partial_data4;
       
       float partial_out1;
       float partial_out2;
       
       partial_data1 = params->par_p6 * params->t_lin;
       partial_data2 = params->par_p7 * (params->t_lin * params->t_lin);
       partial_data3 = params->par_p8 * (params->t_lin * params->t_lin * params->t_lin);
       partial_out1 = params->par_p5 + partial_data1 + partial_data2 + partial_data3;
       
       partial_data1 = params->par_p2 * params->t_lin;
       partial_data2 = params->par_p3 * (params->t_lin * params->t_lin);
       partial_data3 = params->par_p4 * (params->t_lin * params->t_lin * params->t_lin);
       partial_out2 = (float)press_uncomp * (params->par_p1 + partial_data1 + partial_data2 + partial_data3);
       
       partial_data1 = (float)press_uncomp * (float)press_uncomp;
       partial_data2 = params->par_p9 + params->par_p10 * params->t_lin;
       partial_data3 = partial_data1 * partial_data2;
       partial_data4 = partial_data3 + ((float)press_uncomp * (float)press_uncomp * (float)press_uncomp) * params->par_p11;
       
       comp_press = partial_out1 + partial_out2 + partial_data4;
       
       
       
       return comp_press;

}

 

static void bmp384_param(struct bmp384_calib_param* params){ // On lis les paramètres présent dans le bmp384 pour les stocker dans la structure

  uint8_t buff_param[21] = {0};
  uint8_t valP = 0x31;
  
  i2c_write_blocking(i2c_default,addr,&valP,1,true);
  
  i2c_read_blocking(i2c_default, addr, buff_param, 21, false);

  params->nvm_t1 = (buff_param[1] << 8) | buff_param[0];
  params->nvm_t2 = (buff_param[3] << 8) | buff_param[2];
  params->nvm_t3 = buff_param[4];
  
  params->nvm_p1 = (buff_param[6] << 8) | buff_param[5];
  params->nvm_p2 = (buff_param[8] << 8) | buff_param[7];
  params->nvm_p3 = buff_param[9];
  params->nvm_p4 = buff_param[10];
  params->nvm_p5 = (buff_param[12] << 8) | buff_param[11];
  params->nvm_p6 = (buff_param[14] << 8) | buff_param[13];
  params->nvm_p7 = buff_param[15];
  params->nvm_p8 = buff_param[16];
  params->nvm_p9 = (buff_param[18] << 8) | buff_param[17];
  params->nvm_p10 = buff_param[19];
  params->nvm_p11 = buff_param[20];
  
   
  
  params->par_t1 = params->nvm_t1 / pow(2.0,-8);
  params->par_t2 = params->nvm_t2 / pow(2.0,30);
  params->par_t3 = params->nvm_t3 / pow(2.0,48);
  
  params->par_p1 = (params->nvm_p1-pow(2.0,14)) / pow(2.0,20);
  params->par_p2 = (params->nvm_p2-pow(2.0,14)) / pow(2.0,29);
  params->par_p3 = params->nvm_p3 / pow(2.0,32);
  params->par_p4 = params->nvm_p4 / pow(2.0,37);
  params->par_p5 = params->nvm_p5 / pow(2.0,-3);
  params->par_p6 = params->nvm_p6 / pow(2.0,6);
  params->par_p7 = params->nvm_p7 / pow(2.0,8);
  params->par_p8 = params->nvm_p8 / pow(2.0,15);
  params->par_p9 = params->nvm_p9 / pow(2.0,48);
  params->par_p10 = params->nvm_p10 / pow(2.0,48);
  params->par_p11 = params->nvm_p11 / pow(2.0,65);
  
  
  
   
   
   
 }


#endif


 int main() {

 stdio_init_all();

 #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
 #warning i2c/mpu6050_i2c example requires a board with I2C pins
 printf("Default I2C pins were not defined \n");
 #else

 printf("Hello, MPU6050! Reading raw data from registers...\n");

  
 i2c_init(i2c_default, 400 * 1000); // On déclare les broches configurer pour la communication i2c et sa fréquence 400Khz
 gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
 gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
 gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
 gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN,GPIO_FUNC_I2C));

uint32_t pressure; // on déclare deux variables unsigned de 32 bits pour stocker la température et la pression
uint32_t temperature;

bmp_init();

struct bmp384_calib_param params; // on lance une instance appelé params
bmp384_param(&params); // on envoie a la fonction bmp384_param() l'adresse de l'instance de l'object params.

  

while(1){ // Boucle infinie

 bmp_read(&temperature,&pressure); // On lit la pression et la température
  
 float comp_t = bmp384_temp(temperature,&params); // On appele la fonction bmp384_temp et on stocke la valeur ajusté dans une variable de type float comp_t
 float comp_p = bmp384_press(pressure,&params); // On appelle la fonction bmp384_press et on stocke la valeur ajusté dans une variable de type float comp_p
 
 float altitude = (288/0.0065) * (1-pow(comp_p/101700.0,1/5.255)); // on calcule l'altitude avec comme point de référence la mer = 0m
 
 
 // On affiche la température , la presssion en pascal et l'altitude en m. 
 printf("tem = %f\n",comp_t);
 printf("press = %f\n",comp_p);
 printf("altitude = %f\n",altitude);
 
  
 sleep_ms(500); // on laisse un delay de 0.5 s

}


#endif

}
