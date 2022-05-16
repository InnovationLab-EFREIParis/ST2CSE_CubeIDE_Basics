# ST2CSE_CubeIDE_Basics
## Programme de prise en main de la carte Nucleo LG476RG avec CubeIDE

## A savoir 
Dans Core>Src>main.c, l'utilisateur peut seulement �crire du code entre les balises /* USER CODE BEGIN et /* USER CODE END. Sinon il prend le risque que celui soit effac�.
En effet, Basics.ioc est une interface graphique permettant de configurer le microcontroleur, elle g�n�re du code de fa�on automatique  et peut donc modifier le main.c

# GPIO
General purpose Input Outpout. Ce sont les broches du micro controleur qui peuvent �tre utilis�es comme entr�e ou sortie selon leur configuration. On y branche typiquement une LED ou un bouton poussoir.
## Basics.ioc
Clique gauche sur les pin pour les configurer en sorties ou en entr�
Clique droit sur une broche pour lui donner un user_label

## main.h
#define user_label nom_broche
#define B1_Pin GPIO_PIN_13
#define LD2_Pin GPIO_PIN_5
#define D8_Pin GPIO_PIN_9

## main.c
### static void MX_GPIO_Init(void)
Cette fonction g�n�r� automatiquement configure les GPIO


## Drivers>STM32L4xx_HAL_Driver>stm32l4xx_hal_gpio.h
Ce fichier fournit par STMicro d�finit les masques derriere les alias GPIO_PIN_X
