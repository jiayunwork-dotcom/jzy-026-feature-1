# ---- 构建阶段 ----
FROM golang:1.22-alpine AS build

WORKDIR /src

# 先拷依赖描述，利用层缓存
COPY go.mod ./
RUN go mod download

COPY . .

# 纯静态构建，CGO 关闭以匹配 alpine/scratch 运行环境
RUN CGO_ENABLED=0 GOOS=linux go build -trimpath -ldflags="-s -w" -o /out/cam-follower .

# ---- 运行阶段：单容器 ----
FROM golang:1.22-alpine

# 非 root 用户运行
RUN adduser -D -u 10001 appuser && mkdir -p /data && chown appuser:appuser /data
USER appuser

WORKDIR /app
COPY --from=build /out/cam-follower /app/cam-follower

EXPOSE 8080
ENV CAM_ADDR=":8080" CAM_DATA="/data"

# 容器内先跑一遍测试再启动（镜像构建时已编译；这里验证运行二进制健康可用）
HEALTHCHECK --interval=30s --timeout=3s CMD wget -qO- http://127.0.0.1:8080/healthz || exit 1

ENTRYPOINT ["/app/cam-follower"]
