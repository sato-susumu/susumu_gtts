# susumu_gtts
## 機能
gTTSを利用して、テキストから音声を生成するROS2用パッケージです。

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
```bash
ros2 topic pub -1 /tts std_msgs/String "data: 'ピカチュー、1000万ボルト！'"
```

## トラブルシューティング
### 音声が再生されない (Ubuntuの場合)
以下のコマンドで音声デバイスが認識されているか確認してください。  
```bash
lsusb
```
デバイスが認識されていない場合、USB接続のスピーカーを接続してください。  
  
次に以下のコマンドで音声が再生されるか確認してください。  
```bash
aplay /usr/share/sounds/alsa/Side_Right.wav
```
これが再生されない場合、環境設定で音声デバイスが正しく設定されていない可能性があります。  
