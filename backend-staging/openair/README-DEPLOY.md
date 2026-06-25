# Backend OpenAir AirSentinels — déploiement VPS

Cible : VPS Hostinger `185.97.146.160`, dans `/root/openair/`, exposé sur `https://station.airsentinels.fr/`.

## Pré-requis (déjà OK)

- DNS Cloudflare : `station.airsentinels.fr` A `185.97.146.160` DNS only ✓
- Réseau Docker `root_default` existe ✓
- Port host `127.0.0.1:8092` libre ✓
- Traefik avec certresolver `mytlschallenge` ✓
- Espace disque : 89 Go libres sur `/` ✓

## Convention respectée (mirrorée depuis `aq-montpellier`)

- Image `ghcr.io/muchobien/pocketbase:latest`
- Network `root_default` external
- 2 routers Traefik : fallback Hostinger + domaine custom
- Cache-Control no-cache via middleware
- Healthcheck `/api/health`

## Étapes de déploiement (à exécuter après validation David)

### 1. Préparer .env local (NE PAS COMMIT)

```bash
# Sur le VPS, dans /root/openair/
cat > .env <<'EOF'
OPENAIR_DEVICE_TOKEN=<générer 32 chars random>
EOF
chmod 600 .env
```

Générer le token :
```bash
openssl rand -hex 32
```

### 2. Copier les fichiers

Depuis ce dossier local :
```bash
scp -i ~/.ssh/nextpm_vps -r \
  docker-compose.yml pb_migrations pb_hooks pb_public \
  root@185.97.146.160:/root/openair/
```

### 3. Démarrer

```bash
ssh -i ~/.ssh/nextpm_vps root@185.97.146.160
cd /root/openair
docker compose up -d
docker compose logs -f --tail=50
```

Surveiller :
- container `openair-pocketbase` healthy
- Traefik logs : `cert obtained for station.airsentinels.fr`
- `curl -I https://station.airsentinels.fr/api/health` → 200

**En cas d'échec LE** : NE PAS retry en boucle (rate-limit 5 échecs/h). Vérifier d'abord :
- DNS résout depuis le VPS (`dig +short station.airsentinels.fr @1.1.1.1`)
- Le record est bien en DNS only (nuage gris) côté Cloudflare
- Logs Traefik (`docker logs root-traefik-1 --tail 100 | grep -i station`)

### 4. Provisionnement initial

#### Superuser admin
```bash
docker exec -it openair-pocketbase /usr/local/bin/pocketbase superuser create \
  --dir /pb_data david@riallant.com <mot_de_passe_fort>
```
**Piège connu** : `--dir /pb_data` est obligatoire, sinon auth OK puis 400 sur l'admin UI.

#### Test ingestion
```bash
TOKEN=$(grep OPENAIR_DEVICE_TOKEN /root/openair/.env | cut -d= -f2)
curl -X POST https://station.airsentinels.fr/api/openair/ingest \
  -H "X-Device-Token: $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "device_serial": "test000001",
    "ts": "2026-06-25T12:00:00Z",
    "pm1": 1.2, "pm25": 2.3, "pm10": 3.4,
    "co2": 420, "atmp": 21.5, "rhum": 55.0,
    "rssi": -52
  }'
```

Attendu : `{"id":"<recordId>","device":"<deviceId>"}` HTTP 200. Le device `test000001` est créé automatiquement (AUTO_CREATE_DEVICES=true).

### 5. Côté firmware (Phase 1 — pas dans ce déploiement)

Le firmware aura à stocker en NVS :
- `OPENAIR_TOKEN` (Bearer-style, copié depuis le .env du serveur)
- `OPENAIR_URL` = `https://station.airsentinels.fr/api/openair/ingest`
- `DEVICE_SERIAL` = MAC lowercase sans `:` (déjà calculé par `agSerial12()`)

## Backup

À ajouter au cron existant des autres PB :
```
/root/openair/pb_data/ → backup quotidien
```

## Rollback complet

```bash
ssh -i ~/.ssh/nextpm_vps root@185.97.146.160
cd /root/openair
docker compose down
# Optionnel suppression complète :
cd /root && rm -rf openair/
# Cloudflare : supprimer le record station
```

Pas de side-effect sur Traefik, n8n, nextpm-pocketbase, aq-montpellier-pocketbase, traindetector — chacun a son propre compose isolé.
