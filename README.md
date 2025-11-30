# fingerprint-access-control
Access control system using STM32F103 and UART fingerprint reader.

## Description
Ce projet est un système de contrôle d'accès basé sur la reconnaissance d'empreintes digitales, développé sur **STM32F103** avec un module **UART Fingerprint Reader GCO8_V2**.  
Il permet de sécuriser l'accès à un local en enregistrant et en vérifiant les empreintes des utilisateurs.

## Fonctionnalités
- Enregistrement des empreintes digitales via authentification par empreinte de l'admin
- Suppression des empreintes digitales via authentification par empreinte de l'admin
- Vérification des utilisateurs existants
- Communication UART entre le STM32 et le module GCO8_V2
- Retour visuel avec LEDs et retour textuel via Putty

## Matériel Utilisé
- STM32F103C8T6 (Blue Pill)
- Capteur d'empreinte digitale GCO8_V2
- LEDs
- Câbles et alimentation

## Schéma de Connexion
<img width="605" height="546" alt="Schéma de connexion" src="https://github.com/user-attachments/assets/773d9342-e5f6-4df7-bde7-dfa0e4911268" />
