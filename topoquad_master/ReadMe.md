# topoquad_master

## topic

### leg_node
  - /legs/command 脚コマンド（角度/足先座標/力・トルクを統合したメッセージ）
  - /legs/state/present 脚の各関節の角度とトルクの現在の状態
  - /legs/state/goal 脚の各関節の角度とトルクの目標の状態
  - /legs/state/target 脚の各関節の目標コマンド状態

launch では namespace `topoquad` 付きで起動しているので，実際の topic は `/topoquad/legs/...` になる．

### neck_node
 - /neck/command 同上
 - /neck/state/present 同上
 - /neck/state/goal 同上

launch では namespace `topoquad` 付きで起動しているので，実際の topic は `/topoquad/neck/...` になる．

## dynamixel id map

### Leg/脚
`leg_node` のロボット固有値は `config/topoquad_master.yaml` のパラメータとして持つ．
対象は以下。
 - 脚寸法: `legs.link_lengths.*`
 - 脚取付位置/角度: `legs.mounts.<fr|fl|br|bl>.*`
 - 関節特性: `legs.joints.<fr|fl|br|bl>.<joint>.{id,gear_ratio,torque_ratio,default_torque}`
 - 初期姿勢: `legs.initial_pose.<fr|fl|br|bl>`

### Neck/首 (optional)
topoquad_master pkg の neck_node が 持っている情報.
 - Pan : 43
 - Tilt : 42

 launchから書き換え可能．
