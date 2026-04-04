## 🔒 Configuration du Wi-Fi (Réseaux locaux)

Pour des raisons de sécurité, les identifiants et mots de passe Wi-Fi ne sont pas versionnés sur ce dépôt (ils sont ignorés via le `.gitignore`). 

Pour que la connexion Wi-Fi et le Fleet Dashboard fonctionnent sur votre machine ou votre robot, vous devez créer manuellement votre fichier de configuration local.

### ⚙️ Instructions :

1. Dans le dossier `src/`, créez un nouveau fichier nommé exactement **`wifi_credentials.h`**.
2. Copiez-collez le code ci-dessous à l'intérieur.
3. Modifiez les réseaux avec les noms (SSID) et mots de passe de vos propres partages de connexion ou routeurs locaux.

```c
#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

typedef struct {
    const char *ssid;
    const char *password;
} WifiNetwork_t;

// Liste des réseaux à tester (par ordre de priorité)
// Le robot essaiera de s'y connecter un par un au démarrage.
static const WifiNetwork_t wifi_networks[] = {
    {"Nom_Partage_Principal", "MotDePasse123"},
    {"Reseau_Secours", "12345678"},
    // Vous pouvez ajouter d'autres réseaux ici
};

static const int num_wifi_networks = sizeof(wifi_networks) / sizeof(wifi_networks[0]);

#endif // WIFI_CREDENTIALS_H