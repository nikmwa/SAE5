# SAE ESE Wifi BLE - Central

Code pour la carte centrale qui communique avec les périphériques via Bluetooth et transmet les données à la plateforme IoT via HTTP.

### Matériel et logiciel requis
- Kit de prototypage PSoC : CY8CPROTO-062-4343W
- ModusToolbox 3.1

### Guide d'installation

1. Télécharger le dossier **central** à l'emplacement souhaité.
2. Ouvrir Eclipse IDE for ModusToolbox et importer le projet en cliquant sur *Import existing application in place* dans le panneau en bas à gauche de l'application (Quick panel).
3. Connecter la carte PSoC via USB.
4. Modifier les identifiants nécessaires à la connexion wifi et HTTP dans le fichier 'http_client.h' : **WIFI_SSID**, **WIFI_PASSWORD**, **WIFI_SECURITY_TYPE**, **HTTP_PORT**, **HTTP_SERVER_HOST**, **HTTP_PATH**.
5. Modifier le nom de l'appareil périphérique dans 'app_bt.c' : **SEARCH_DEVICE_NAME**.
6. Lancer la compilation du code et la programmation de la carte en appuyant sur le bouton *Run* (si une boîte de dialogue apparaît, choisir *Program*). Si la programmation échoue, assurez-vous que le programmateur de la carte soit à jour : ouvrir l'application **Cypress Programmer**. À l'ouverture, l'application devrait avertir de l'obsolescence du programmateur. Cliquer sur le bouton *Upgrade Firmware* dans la boîte de dialogue qui apparaît.
7. Une communication UART est nécessaire pour vérifier le statut de la connexion bluetooth et de la connexion HTTP (par exemple via TeraTerm). Baud = 115200, flow control = Xon/Xoff. Au démarrage, la centrale se connecte au réseau wifi, et s'il est accessible, au serveur HTTP de la plateforme Thingsboard.
8. La connexion au périphérique bluetooth doit se faire manuellement :
   1. Lancer le scan d'appareils en envoyant 's' sur l'interface série. L'appairage devrait s'effectuer automatiquement.
   2. Lancer la découverte des services en envoyant 'q' sur l'interface série.
   3. Lancer la découverte des caractéristiques en envoyant 'w' sur l'interface série.
   4. Lancer la découverte des descripteurs en envoyant 'e' sur l'interface série.
9. Maintenant que le périphérique bluetooth est connecté, plusieurs commandes sont possibles :
   - '0' ou '7' pour éteindre ou allumer la LED utilisateur du périphérique
   - 'r' pour lire l'état de la LED utilisateur
   - 'n' pour s'abonner au nombre d'appuis sur le bouton : cela enclenche l'envoi à la plateforme du nombre d'appuis via une requête HTTP 'POST'
   - 'm' pour envoyer une requête HTTP 'POST' de test

### Pistes d'amélioration 

- Automatiser la connexion bluetooth et la découverte de services/caractéristiques au travers d'une machine à états.
- Gérer la connexion bluetooth de plusieurs périphériques.
- Gérer la transmission de la lecture de donnée provenant du capteur associé au périphérique (bouton d'abonnement + remontée des données notifiées).
- Utiliser un autre protocole pour remonter les données au serveur (MQTT).
