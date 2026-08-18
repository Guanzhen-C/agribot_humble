# Agribot semantic Neo4j containers

This deployment keeps the indoor and outdoor semantic graphs in separate
Neo4j 5.26 containers on `172.18.80.26`:

- outdoor: HTTP `7476`, Bolt `7689`
- indoor: HTTP `7478`, Bolt `7691`

All persistent database, log and import files are bind-mounted below
`/data/agribot` by default. Docker's shared image layer remains in Docker's
global data root; the semantic graph data never consumes the system disk.

Copy `.env.example` to `.env`, set different strong passwords, create the bind
mount directories, then run:

```bash
mkdir -p /data/agribot/neo4j/{data,logs,import} \
  /data/agribot/neo4j-indoor/{data,logs,import}
docker compose up -d
docker compose ps
```

These four database ports remain blocked by the host firewall. The local
semantic service reaches them through `127.0.0.1`; phones access only the
bounded API on `8090/tcp`. Do not publish the `.env` file or place a password
in a ROS launch argument or APK.
