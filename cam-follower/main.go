// cam-follower 是凸轮从动件运动学小服务：
// 经 HTTP 提供余弦/摆线/等加速等减速三条规律的升程曲线，
// 以及“升程—远休止—回程—近休止”整周循环曲线，并核对各接头连续性。
package main

import (
	"context"
	"errors"
	"flag"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"camfollower/internal/kinematics"
	"camfollower/internal/law"
	"camfollower/internal/store"
)

func main() {
	addr := flag.String("addr", envOr("CAM_ADDR", ":8080"), "HTTP 监听地址")
	dataDir := flag.String("data", envOr("CAM_DATA", "data"), "配方与循环档目录")
	flag.Parse()

	st, err := store.Open(*dataDir)
	if err != nil {
		log.Fatalf("打开档目录失败: %v", err)
	}
	if err := seed(st); err != nil {
		log.Fatalf("载入初始配方失败: %v", err)
	}

	srv := &http.Server{
		Addr:              *addr,
		Handler:           routes(st),
		ReadHeaderTimeout: 10 * time.Second,
	}

	go func() {
		log.Printf("cam-follower 监听 %s（角度单位：%s，ω 单位：%s）", *addr, kinematics.AngleUnit, kinematics.OmegaUnit)
		if err := srv.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
			log.Fatalf("HTTP 服务失败: %v", err)
		}
	}()

	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)
	<-stop
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_ = srv.Shutdown(ctx)
}

// seed 启动时载入摆线升程配方一条：两端加速度为 0，终点位移等于 h。
func seed(st *store.Store) error {
	const name = "cycloid-lift"
	if _, err := st.GetRecipe(name); err == nil {
		return nil // 已存在则保留调用方改过的版本
	} else if !errors.Is(err, store.ErrNotFound) {
		return err
	}
	r := store.Recipe{Name: name, Type: law.Cycloid, H: 10, Beta: 60}
	if err := st.SaveRecipe(r); err != nil {
		return err
	}
	log.Printf("已载入初始配方 %q：类型=%s h=%g β=%g°", name, r.Type, r.H, r.Beta)
	return nil
}

func envOr(key, def string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return def
}
