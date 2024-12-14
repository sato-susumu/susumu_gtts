# susumu_gtts
## 機能
gTTSを利用して、テキストから音声を生成するノードです。


## 音声再生テスト
```
ros2 topic pub -1 /tts std_msgs/String "data: '次の音声を再生します。'"
```