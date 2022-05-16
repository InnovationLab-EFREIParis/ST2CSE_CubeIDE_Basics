# ST2CSE_CubeIDE_Basics
## Programme de prise en main de la carte Nucleo LG476RG avec CubeIDE

**A savoir**
Dans Core>Src>main.c, l'utilisateur peut seulement écrire du code entre les balises /* USER CODE BEGIN et /* USER CODE END. Sinon il prend le risque que celui soit effacé.
En effet, Basics.ioc est une interface graphique permettant de configurer le microcontroleur, elle génére du code de façon automatique et peut donc modifier le main.c.
Les fonctions commençant par le préfixe HAL (Hardware Abstraction Layer) permettent de configurer et d'utiliser les ressources du microcontroler. Elles sont fournient par STMicroElectronics. Elles sont dans le dossier Drivers>STM32L4XX_HAL_Driver.
Les fonctions fournient par ARM n'ont pas specialement d'identifiant, elles se trouvent dans Drivers>CMSIS


# GPIO
General purpose Input Outpout. Ce sont les broches du micro controleur qui peuvent être utilisées comme entrée ou sortie selon leur configuration. On y branche typiquement une LED ou un bouton poussoir.
Il y a 3 ports (A, B, C) comportant chacun 16 broches. Par exemple, PA8 est la 8e broche du port A. Il y a theoriquement 48 GPIO de disponible sur notre carte.

## Basics.ioc
Clique gauche sur les broches pour les configurer en sorties ou en entré<br>
Clique droit sur une broche pour lui donner un user_label

## main.h
Les directives suivantes permettent de faire le lien entre les user_label et les noms des broches<br>
#define user_label nom_broche<br>
#define B1_Pin GPIO_PIN_13<br>
#define LD2_Pin GPIO_PIN_5<br>
#define D8_Pin GPIO_PIN_9

## main.c
### static void MX_GPIO_Init(void)
Cette fonction généré automatiquement configure les GPIO

### static void MX_GPIO_Init(void)
Cette fonction généré automatiquement configure les GPIO

### void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
exemple: HAL_GPIO_WritePin(GPIOA, LD2_Pin, RESET);

### GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
Exemple: PinState = HAL_GPIO_ReadPin(GPIOC, B1_Pin);

## Drivers>STM32L4xx_HAL_Driver>stm32l4xx_hal_gpio.h
Ce fichier fournit par STMicro définit les masques derriere les alias GPIO_PIN_X ainsi que toutes les fonctions relatives à l'utilisation des GPIO
