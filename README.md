# susumu_gtts
## 機能
gTTSを利用して、テキストから音声を生成するノードです。

## ノード起動
再生速度1.5倍、日本語で起動
```
ros2 run susumu_gtts susumu_gtts
```

再生速度1.5倍、英語で起動
```
ros2 run susumu_gtts susumu_gtts --ros-args -p lang:=en -p playback_speed:=1.0
```

## パラメータ変更
言語を英語に変更
```
ros2 param set /susumu_gtts lang en
```

再生速度を1.0倍に変更(デフォルトは1.5倍)
```
ros2 param set /susumu_gtts playback_speed 1.0
```

## 音声再生テスト
```
ros2 topic pub -1 /tts std_msgs/String "data: 'ピカチュー、1000万ボルト！'"
```