/*
 *  Fichier permettant la configuration de la canne
 */

#ifndef PCB_CANNEASON_BOARDCONFIG_H
#define PCB_CANNEASON_BOARDCONFIG_H

/*  Define permettant d'activer ou non les couleurs de debug sur l'écran  */
//#define DEBUG_SCREEN

/*  Définition de la fréquence du microcontrôleur  */
#define UC_FREQUENCY        100000UL

/*  Paramétrage des capteurs ultrasons CH201  */
#define NB_CH201_SENSOR     2                           // Choix du nombre de capteurs utilisés
#define CH201_FIRMWARE      ch201_gprmt_init            // Firmware des capteurs (ici le mode Multi Thresholds)
#define CH201_MODE          CH_MODE_TRIGGERED_TX_RX     // Mode de capture (CH_MODE_FREERUN ou CH_MODE_TRIGGERED_TX_RX)
#define CH201_MAX_RANGE     4000                        // Distance mesurable maximum en mm
#define CH201_MEASUREMENT_INTERVAL	100	                // Période d'échantillonnage, en ms. Période de 100ms = fréquence d'échantillonnage de 10Hz

/*  Déclaration des Entrées/Sorties  */
#define BUZZER_PIN          26
#define BUTTON              39
#define PIN_RESET           23
#define PIN_SDA             25
#define PIN_SCL             21
#define INT1                22
#define INT2                33
#define PROG1               19
#define PROG2               32

/*  Paramétrage du retour à l'utiliateur via le buzzer  */
#define NB_BIP              4       // Nombre de cas différents à traiter (ici 4)
#define VOLUME_BIP          500     // Volume des bips (entre 0 et 512)
#define BIP_OBSTACLE        2500    // Fréquence bip en cas d'obstacle en Hz
#define BIP_ESCALIER        5000    // Fréquence bip en cas d'escalier/trou en Hz
#define BIP_OBSTACLE_HAUTEUR 800    // Fréquence bip en cas d'obstacle en hauteur en Hz

extern hw_timer_t *My_timer;
#endif //PCB_CANNEASON_BOARDCONFIG_H
