<h1>esp32 vst</h1>

measとcommを一体化

# comポート設定

1: 自動検出（Auto-detect）に任せる

; upload_port = com6
; monitor_port = com6

2: ターミナルの環境変数で指定する

ターミナルで PlatformIO 用の環境変数をセットしておくと platformio.ini より優先される。

```
$env:PLATFORMIO_UPLOAD_PORT = "COM6"
$env:PLATFORMIO_MONITOR_PORT = "COM6"
```

```
Remove-Item Env:\PLATFORMIO_UPLOAD_PORT
Remove-Item Env:\PLATFORMIO_MONITOR_PORT
```
