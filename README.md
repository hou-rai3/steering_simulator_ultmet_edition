## ステアリングシミュレーターUltimate Edition
Here’s a structured explanation of the code:

---

### **概要（Overview）**
本コードは、PID制御を用いた車両の独立操舵シミュレーションを実装したものです。`pygame` を使用してシミュレーションを描画し、`PySide6` を用いたGUIによってリアルタイムでPIDゲインを調整できるようになっています。ユーザーはマウスの動きによって目標位置を設定でき、車両はPID制御を用いてその目標位置に向かうように操舵および速度制御を行います。

---

### **システムの構成**
このシステムは、以下の2つのプロセスから構成されています。

1. **GUIプロセス**（`run_gui`）  
   `PySide6.QtWidgets` を用いてPIDゲイン調整用のスライダーを提供します。ユーザーはこのスライダーを動かすことで、リアルタイムにPID制御のパラメータを調整できます。

2. **シミュレーションプロセス**（`run_simulation`）  
   `pygame` を使用して車両の動きをシミュレーションします。PID制御によって車両の角度および速度を調整し、マウスで指定された目標位置に向かうように制御を行います。

両プロセス間のデータ共有は `multiprocessing.Manager().dict()` を用いた共有辞書 `shared_data` によって行われ、PIDパラメータの更新がリアルタイムで反映されます。

---

### **PID制御について**
PID制御は、目標値と現在の値との差（誤差）を基にして適切な制御量を決定する手法です。本コードでは、角度制御と速度制御の2つのPID制御が導入されています。

#### **1. 角度制御**
車両が目標位置に向かうように、以下の式で角度を調整します。

\[
\text{PID出力} = K_p \times \text{誤差} + K_i \times \sum \text{誤差} + K_d \times \frac{\Delta \text{誤差}}{\Delta t}
\]

ここで、  
- `誤差 = 目標角度 - 現在の角度`
- `積分項` は誤差の累積値（長期的なズレを補正）
- `微分項` は誤差の変化率（急激な変化を抑制）

このPID出力によって `left_wheel_angle` が調整されます。

#### **2. 速度制御**
目標速度と現在の速度の差を基に、同様のPID制御を行います。

- `誤差 = 目標速度 - 現在の速度`
- `積分項` は速度誤差の累積
- `微分項` は速度誤差の変化率

PIDの出力を `left_wheel_speed` に加算することで、適切な速度制御を行います。

---

### **GUIについて**
`PySide6.QtWidgets` を用いたGUIは、6つのスライダーを提供します。

- **角度制御用のPIDゲイン**
  - KP Angle（比例ゲイン）
  - KI Angle（積分ゲイン）
  - KD Angle（微分ゲイン）

- **速度制御用のPIDゲイン**
  - KP Speed（比例ゲイン）
  - KI Speed（積分ゲイン）
  - KD Speed（微分ゲイン）

各スライダーを動かすことで、`shared_data` に格納されたPIDパラメータが更新され、シミュレーションにリアルタイムで反映されます。

---

このシステムにより、PIDゲインの調整が視覚的に確認でき、チューニングの影響を直感的に理解することが可能になります。
