import math
import time
import serial
import os
import csv
import socket, struct



#########################################################
RB_PORT = "COM3"  # シリアルポート設定
#########################################################

# ルンバ制御設定
BAUD_RATE = 115200 # 通信速度
# BAUD_RATE = 19200  # 通信速度
START_OP_CODE = 128  # Start コマンド
FULL_MODE_OP_CODE = 132  # Full モードコマンド

# 車輪の設定
WHEEL_DIAMETER = 0.072  # 車輪の直径 [m]
COUNTS_PER_REVOLUTION = 508.8  # 1回転あたりのエンコーダカウント数
TICKS_PER_REVOLUTION = 65535  # 16bit エンコーダーの最大値（1回転）

# 直進設定
PWM = 50  # 直進時のPWMの大きさ
DRIVE_TIME = 5  # 直進時間 [秒]

#実験設定
MinPWM=-1*PWM #実験項目2のPWMの大きさ
AddPWM=PWM-35 #実験項目3のPWMの大きさ
TEST_TIME_1=5 #実験項目2の旋回開始時間[秒]
TEST_TIME_2=5.59 #実験項目2の旋回終了時間[秒]

RB_LEFT_ENC = 43  # 左エンコーダカウント
RB_RIGHT_ENC = 44  # 右エンコーダカウント

def Set_UDP_Sender(IP_src, Port_src, IP_dst, Port_dst):

    Address_src = (IP_src, Port_src)
    Address_dst = (IP_dst, Port_dst)

    Socket_Sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    Socket_Sender.bind(Address_src)



        # 送信者の Socket オブジェクトと送信先のアドレスを返す
    return Socket_Sender, Address_dst


def _CalcHLByte(n):
    HB = n & 0xff00
    HB >>= 8
    LB = n & 0x00ff
    return (HB, LB)

def DrivePWM(ser, L_PWM, R_PWM):
    L_HB, L_LB = _CalcHLByte(L_PWM)
    R_HB, R_LB = _CalcHLByte(R_PWM)
    ser.write(bytes([146, R_HB, R_LB, L_HB, L_LB]))

def GetSensor(ser, p_id, len, sign_flg):
    ser.write(bytes([142, p_id]))
    ser.flushInput()
    data = ser.read(len)
    return int.from_bytes(data, "big", signed=sign_flg)

def GetEncs(ser):
    EncL = GetSensor(ser, RB_LEFT_ENC, 2, False)
    EncR = GetSensor(ser, RB_RIGHT_ENC, 2, False)
    return (EncL, EncR)

def initialize_roomba(ser):
    # ルンバの初期化
    ser.write(bytes([START_OP_CODE]))  # スタートモード
    time.sleep(0.1)  # 少し待機
    ser.write(bytes([FULL_MODE_OP_CODE]))  # フルモード
    time.sleep(0.1)

def correct_encoder_overflow(current_ticks, prev_ticks, ticks_per_revolution):

    delta_ticks = current_ticks - prev_ticks

    # オーバーフロー検出と補正
    if delta_ticks > ticks_per_revolution / 2:  # 正方向のオーバーフロー
        delta_ticks -= ticks_per_revolution
    elif delta_ticks < -ticks_per_revolution / 2:  # 負方向のオーバーフロー
        delta_ticks += ticks_per_revolution

    return delta_ticks

def main():

    


    ser = serial.Serial(RB_PORT, BAUD_RATE, timeout=10)


    
    # ルンバを初期化し、フルモードに設定
    initialize_roomba(ser)
    # 初期化
    start_time = time.time()
    last_time = start_time

    
    
    
    prev_left_enc, prev_right_enc = GetEncs(ser)
    # 初期位置と角度
    x, y, theta = 0.0, 0.0, 0.0
    Sender, Destination = Set_UDP_Sender("127.0.0.1", 5000, "127.0.0.1", 4000)

    # ルンバを直進
    DrivePWM(ser, PWM-3, PWM)  # 直進しながら計測を開始

    #ルンバを旋回
    #DrivePWM(ser, PWM-3, MinPWM) #左方向
    #DrivePWM(ser, MinPWM, PWM) #右方向

    #左右の速度差がある場合
   
    #DrivePWM(ser, PWM-4, AddPWM)

    #DrivePWM(ser, AddPWM, PWM)
    
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        delta_time = current_time - last_time

        #if TEST_TIME_1 <= elapsed_time and elapsed_time <= TEST_TIME_2:
        #    DrivePWM(ser, PWM-4, MinPWM)  # 停止
            # print("Rotating motion")
        #elif TEST_TIME_2 < elapsed_time and elapsed_time<DRIVE_TIME:
        #    DrivePWM(ser, PWM-4, PWM) 
            # print("Drive motion")


        # DRIVE_TIMEを超えたら停止してループを抜ける
        if elapsed_time >= DRIVE_TIME:
            DrivePWM(ser, 0, 0)  # 停止
            print("Drive complete.")
            break

        # delta_timeがゼロの場合をチェック
        if delta_time <= 1e-6:  # 極小値でスキップ
            last_time = current_time
            continue

        # エンコーダー値取得
        left_enc, right_enc = GetEncs(ser)
        if left_enc is None or right_enc is None:
            print("Error: Encoder values are invalid. Skipping this iteration.")
            continue

        # エンコーダのパルス差分を計算
        delta_left = correct_encoder_overflow(left_enc, prev_left_enc, TICKS_PER_REVOLUTION)
        delta_right = correct_encoder_overflow(right_enc, prev_right_enc, TICKS_PER_REVOLUTION)

        # 車輪ごとの角速度を計算し、線速度を求める
        velocity_left = (2 * math.pi * delta_left / COUNTS_PER_REVOLUTION) * (WHEEL_DIAMETER / 2) / delta_time
        velocity_right = (2 * math.pi * delta_right / COUNTS_PER_REVOLUTION) * (WHEEL_DIAMETER / 2) / delta_time
        velocity_ave = (velocity_left + velocity_right) / 2
        velocity_angular = (velocity_right - velocity_left) / 0.235

        # 離散積分で位置と角度を更新
        theta += velocity_angular * delta_time
        x += velocity_ave * math.cos(theta) * delta_time
        y += velocity_ave * math.sin(theta) * delta_time

        # 結果を出力
        print(f"Time: {elapsed_time:.3f} s | Position: ({x:.2f}, {y:.2f}) | "
              f"Theta: {math.degrees(theta):.2f} deg | "
              f"Velocity Ave: {velocity_ave:.2f} m/s | "
              f"Velocity Ang: {velocity_angular:.2f} rad/s")

        # 前回のエンコーダー値と時刻を更新
        prev_left_enc = left_enc
        prev_right_enc = right_enc
        last_time = current_time

        #UDP通信で送信
        Send_Data_Binari = struct.pack( "fffffff",
                                        elapsed_time,
                                        x,
                                        y,
                                        theta,
                                        math.degrees(theta),
                                        velocity_ave,
                                        velocity_angular
                                        )

            #print("vol:"+str(el)+"cur:"+str(er))

        Sender.sendto( Send_Data_Binari, Destination )

        # データを保存

        time.sleep(0.1)  # 0.1秒待機

main()
