# 手机语义规划服务

该服务部署在 `172.18.80.26`，手机通过 VPN 调用 `8090/tcp`。服务在服务器内部访问：

- 阿里百炼 `qwen3.7-flash`：解析自然语言并选择 Neo4j 候选地点；
- 阿里百炼 `text-embedding-v4`：生成 1024 维查询向量；
- 室外 Neo4j `7476` 与室内 Neo4j `7478`。

手机只拿到按用户顺序排列的目标地点和明确禁止通行的地点。RDK把目标交给Nav2 Smac，把禁行地点写入Keepout Filter；服务端不运行拓扑A*，也不生成路线偏好代价。服务器不能直接控制底盘。百炼 API Key 和 Neo4j 密码只保存在服务器的 `semantic.env`，不得写入 APK、网页资源或 Git。

部署目录统一放在数据盘：

```bash
mkdir -p /data/agribot/semantic_service /data/agribot/semantic_tasks
cp config.example.json /data/agribot/semantic_service/config.json
chmod 600 /data/agribot/semantic_service/config.json \
  /data/agribot/semantic_service/semantic.env
```

`semantic.env` 必须包含 `DASHSCOPE_API_KEY` 以及两张地图各自的 Neo4j 密码。

防火墙只需允许手机 VPN 出口访问 `8090/tcp`。不要向手机开放 Neo4j 的 `7476/7478/7689/7691` 或 Ollama 的 `11434`。
