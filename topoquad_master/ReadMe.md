# topoquad_master

## topic

### leg_node
  - /legs/angle 脚の各関節の角度を直接指定．
  - /legs/point ボディ座標系から見た足先の座標を指定
  - /legs/state/present 脚の各関節の角度とトルクの現在の状態
  - /legs/state/goal 脚の各関節の角度とトルクの目標の状態

launchファイルから起動した場合以下のようにremapされているので注意
``` xml
  <remap from="/legs/angle" to="/spider/cmd/leg_angle" />
  <remap from="/legs/point" to="/spider/cmd/leg_point" />
  <remap from="/legs/state/present" to="/spider/state/leg/present" />
  <remap from="/legs/state/goal" to="/spider/state/leg/goal" />
```

### neck_node
 - /neck/angle 同上
 - /neck/state/present 同上
 - /neck/state/goal 同上

launchファイルから起動した場合以下のようにremapされているので注意
```xml
  <remap from="/neck/angle" to="/spider/cmd/neck_angle" />
  <remap from="/neck/state/present" to="/spider/state/neck/present" />
  <remap from="/neck/state/goal" to="/spider/state/neck/goal" />
```
