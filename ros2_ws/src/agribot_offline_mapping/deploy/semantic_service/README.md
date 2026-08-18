# 手机语义规划服务

该服务部署在 `172.18.80.26`，手机通过 VPN 调用 `8090/tcp`。服务在服务器内部访问：

- Ollama `qwen3.8:27b`：解析自然语言并选择 Neo4j 候选地点；
- Ollama `qwen3-embedding:8b`：生成 4096 维查询向量；
- 室外 Neo4j `7476` 与室内 Neo4j `7478`；
- Dijkstra：只在已验收的 `DRIVABLE` 拓扑边上生成确定性路线。

手机拿到的只是预览路线。随后手机把整条路线提交给 RDK 的受控接口，服务器不能直接控制底盘。Neo4j 密码只保存在服务器的 `semantic.env`，不得写入 APK、网页资源或 Git。

部署目录统一放在数据盘：

```bash
mkdir -p /data/agribot/semantic_service /data/agribot/semantic_tasks
cp config.example.json /data/agribot/semantic_service/config.json
chmod 600 /data/agribot/semantic_service/config.json \
  /data/agribot/semantic_service/semantic.env
```

防火墙只需允许手机 VPN 出口访问 `8090/tcp`。不要向手机开放 Neo4j 的 `7476/7478/7689/7691` 或 Ollama 的 `11434`。
