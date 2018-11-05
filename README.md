# ROS visualization_helper package

ROSで可視化をする時に便利な道具をまとめたパッケージ．
[jsk-visualization](https://github.com/jsk-ros-pkg/jsk_visualization)と組み合わせて使うことを想定している．

# topic_translator ノード

rostopic list で表示される topic ならなんでも読んで分解して std_msgs/float32 に変換するためのノード．
jsk-visualization の rviz プラグインのグラフ表示が float32 しか受け付けないため作った．

## 使い方

```
rosrun visualization_helper topic_translator.py _input:="/topic_name/to/read" 
```

```~input``` パラメータに文字列で読みたいトピック名を与える．
このトピック名は，トピック名にその要素を付記することで細かい指定を行う．

例 : オドメトリ(/ypspur/odom topic)のx方向移動成分を読みたい場合

```
/ypspur/odom/twist/twist/linear/x
```

実際にどのように要素を記述すればいいかは，topicによるので，rostopic echo で表示された値をよく読んで確認すること．

```~gain``` 読んだパラメータにゲインをかける．
例えば入力が±1.0に変化する場合，gainを100にすれば±100と変化するようになるので，可視化の際に正規化した値をパーセントとして読みたい場合などに有効．
また，角度のラジアン・度変換にも利用できる．

# quaternion_translator ノード

rostopic list で表示される topic のうち，クオータニオン表記があるもの解釈してオイラー角に変換し，std_msgs/float32 でpublishし直すためのノード．

## 使い方

```
rosrun visualization_helper quaternion_translator.py _input:="/topic_name/to/read"
```

```~input``` パラメータに文字列で読みたいトピック名を与える．
このトピック名は，クオータニオン表記になっているところまで分解して記述する．

例 : IMU (/imu/data topic)の姿勢情報を読みたい場合

```
/imu/data/orientation
```


