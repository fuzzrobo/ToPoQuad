# topoquad_master

## launch

- `ros2 launch topoquad_master topoquad_master.launch.py model:=topoquad`
- `model` 未指定時は環境変数 `topoquad_model` を参照し，それも無ければ `topoquad` を使います．
- モデル差分は `config/models/<model>.yaml` にまとめ，`leg_node/neck_node` と `dxl_handler` の両方へ overlay として渡します．

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
`leg_node` の共通設定は `config/topoquad_master.yaml`，モデル固有値は `config/models/<model>.yaml` に持つ．
対象は以下.
 - 脚寸法: `legs.link_lengths.*`
 - 脚取付位置/角度: `legs.mounts.<fr|fl|br|bl>.{position_polar,yaw_deg}` (`position_polar=[radius_m, theta_deg]`)
 - 関節特性: `legs.joints.<fr|fl|br|bl>.<joint>.{id,gear_ratio,torque_ratio,default_torque}`
 - 初期姿勢: `legs.initial_pose_deg.<fr|fl|br|bl>`

### Neck/首 (optional)
topoquad_master pkg の neck_node が 持っている情報.
 - Pan : 43
 - Tilt : 42

 launchから書き換え可能．
