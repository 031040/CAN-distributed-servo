#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN 伺服控制系统 —— Python 上位机（实时绘图）

功能：
  1. 串口接收 STM32 发来的数据
  2. 实时绘制 位置/速度/PWM 三条曲线
  3. 支持在运行中输入指令，在线调整 PID 参数

板端数据格式（每行一条，逗号分隔，\r\n 结尾）：
  pos,target_pos,speed,target_speed,pwm

用法：
  python main_plot.py                # 自动查找 CH340 串口
  python main_plot.py --port COM3    # 指定串口
  python main_plot.py --baud 57600   # 指定波特率

运行中可输入的命令（回车发送）：
  Kp 0.5     设置位置/速度环 Kp
  Ki 0.1     设置 Ki
  Kd 0.01    设置 Kd
  pos 500    设置目标位置
  spd 50     设置目标速度
  pwm 30     直接设置 PWM
  q          退出
"""

import sys
import argparse
import threading
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt

# 串口数据缓冲长度（最多显示最近 600 个采样点，约 12 秒）
MAXLEN = 600


def find_ch340_port():
    """自动查找 CH340 / USB-SERIAL 串口"""
    for port in serial.tools.list_ports.comports():
        desc = port.description
        if "CH340" in desc or "USB-SERIAL" in desc or "USB Serial" in desc:
            return port.device
    return None


def open_serial(port, baud):
    """打开串口，失败时给出友好提示"""
    try:
        return serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"[错误] 打不开串口 {port}：{e}")
        print("  可能原因：")
        print("    1. 串口被其他程序占用（先关掉 CCcom / 串口助手）")
        print("    2. CH340 没插好或没识别")
        print("    3. 换个 COM 号试试（设备管理器里看）")
        sys.exit(1)


def read_loop(ser, data, stop_event):
    """后台线程：持续读串口，解析数据"""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except Exception:
            break
        if not raw:
            continue
        text = raw.decode(errors="ignore").strip()
        parts = text.split(",")
        if len(parts) < 5:
            continue
        try:
            vals = [float(p) for p in parts[:5]]
        except ValueError:
            continue  # 收到乱码/半截行，跳过
        data["x"].append(data["n"][0])
        data["n"][0] += 1
        data["pos"].append(vals[0])
        data["t_pos"].append(vals[1])
        data["speed"].append(vals[2])
        data["t_speed"].append(vals[3])
        data["pwm"].append(vals[4])


def command_thread(ser, stop_event):
    """后台线程：读取键盘输入，发串口指令"""
    print("\n[提示] 输入指令后回车发送。输入 q 退出。")
    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        if cmd.lower() == "q":
            stop_event.set()
            break
        # 所有指令以 \r\n 结尾发送（和板端约定一致）
        ser.write((cmd + "\r\n").encode())
        print(f"  -> 已发送: {cmd}")


def setup_figure():
    """创建 3 个子图：位置 / 速度 / PWM"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax1.set_title("位置 / 目标位置")
    (l_pos,) = ax1.plot([], [], "b-", label="实际位置")
    (l_tpos,) = ax1.plot([], [], "r--", label="目标位置")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    ax2.set_title("速度 / 目标速度")
    (l_spd,) = ax2.plot([], [], "b-", label="实际速度")
    (l_tspd,) = ax2.plot([], [], "r--", label="目标速度")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    ax3.set_title("PWM 输出")
    (l_pwm,) = ax3.plot([], [], "g-", label="PWM")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right")
    ax3.set_xlabel("采样点")

    plt.tight_layout()
    return fig, (ax1, ax2, ax3), (l_pos, l_tpos, l_spd, l_tspd, l_pwm)


def update_plot(axes, lines, data):
    """把缓冲数据画到图上"""
    (ax1, ax2, ax3) = axes
    (l_pos, l_tpos, l_spd, l_tspd, l_pwm) = lines
    x = list(data["x"])

    l_pos.set_data(x, list(data["pos"]))
    l_tpos.set_data(x, list(data["t_pos"]))
    l_spd.set_data(x, list(data["speed"]))
    l_tspd.set_data(x, list(data["t_speed"]))
    l_pwm.set_data(x, list(data["pwm"]))

    # 稳定纵轴：只在数据超出当前范围时才扩展，绝不缩小。
    # 否则电机一停 PWM=0 时整幅图会放大到 ±1，再给命令又缩回 ±49，画面一直跳。
    for ax in axes:
        old_ylo, old_yhi = ax.get_ylim()
        ax.relim()
        ax.autoscale_view()
        new_ylo, new_yhi = ax.get_ylim()
        ax.set_ylim(min(old_ylo, new_ylo), max(old_yhi, new_yhi))


def main():
    parser = argparse.ArgumentParser(description="STM32 实时上位机")
    parser.add_argument("--port", help="串口号，如 COM3")
    parser.add_argument("--baud", type=int, default=57600, help="波特率，默认 57600")
    args = parser.parse_args()

    # 1. 找串口
    port = args.port or find_ch340_port()
    if not port:
        print("[错误] 没找到 CH340 串口。")
        print("  请先：")
        print("    1. 把 CH340 插到电脑 USB")
        print("    2. 确认设备管理器里能看到 COM 口")
        print("    3. 用 --port COM3 手动指定")
        sys.exit(1)

    # 2. 打开串口
    ser = open_serial(port, args.baud)
    print(f"[信息] 串口已打开：{port} @ {args.baud}")
    print("[信息] 等待 STM32 数据...")
    print("[提示] 板端波特率必须是 57600（CubeMX 里 USART1 改），两边不一致会乱码")

    # 3. 初始化数据缓冲
    data = {
        "x": deque(maxlen=MAXLEN),
        "pos": deque(maxlen=MAXLEN),
        "t_pos": deque(maxlen=MAXLEN),
        "speed": deque(maxlen=MAXLEN),
        "t_speed": deque(maxlen=MAXLEN),
        "pwm": deque(maxlen=MAXLEN),
        "n": [0],  # 采样计数（用列表方便线程内修改）
    }

    # 4. 启动两个后台线程：读数据 + 收指令
    stop_event = threading.Event()
    t_read = threading.Thread(target=read_loop, args=(ser, data, stop_event), daemon=True)
    t_cmd = threading.Thread(target=command_thread, args=(ser, stop_event), daemon=True)
    t_read.start()
    t_cmd.start()

    # 5. 建立绘图窗口
    fig, axes, lines = setup_figure()
    plt.ion()  # 交互模式
    fig.show()

    # 6. 主循环：每 5 个采样点刷新一次图
    redraw_cnt = 0
    try:
        while not stop_event.is_set():
            # 用户直接关掉绘图窗口时自动退出
            if not plt.fignum_exists(fig.number):
                print("\n[信息] 绘图窗口已关闭，退出。")
                break
            redraw_cnt += 1
            if redraw_cnt >= 5 and len(data["x"]) > 0:
                update_plot(axes, lines, data)
                plt.pause(0.01)
                redraw_cnt = 0
            else:
                plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()
        print("\n[信息] 已退出，串口已关闭。")


if __name__ == "__main__":
    main()
