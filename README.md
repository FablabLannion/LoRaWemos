# LoRaWemos
Contient du code pour module Lora Wemos

## TTGOGps
Connection TTGO T3 v1 et GPS NEO-M8

Bibliothèques nécessaires (à installer via Manage Libraries) :
* MCCI LoRaWAN LMIC library v2.2.2
* U8g2 v2.24.3

Connection GPS :
* GPS_TX 23 // gps tx to uart rx
* GPS_RX 22 // gps rx to uart tx
* Vcc : 3.3V

## TTGOLoRaOledGps
Communication TTGO T3 v1 en LoRa OTAA et affichage d'infos sur l'écran oled

(pas de gestion GPS malgès lme nom du prtojet :) )

Bibliothèques nécessaires (à installer via Manage Libraries) :
* MCCI LoRaWAN LMIC library v2.2.2
* U8g2 v2.24.3

*Note:* Copier le fichier keys.h.template en keys.h et renseigner les valeurs nécessaires à partir de la console ttn

## LoraAPB.ino
Communication TTGO T3 v1 en LoRa APB et affichage d'infos sur l'écran oled

## LoraOTAA.ino
Communication TTGO T3 v1 en LoRa OTAA et affichage d'infos sur l'écran oled

## TTGO.py
Exemple de récupération des messages uplink en mqtt/python

à remplacer dans le fichier :
* MQTT_HOST="eu.thethings.network" # host du mqtt
* MQTT_TOPIC="+/devices/+/up" # topic pour les messages uplink
* APPID="wemostangi" # id de l'application TTN
* PSW="ttn-account-v2.A_CHANGER"



