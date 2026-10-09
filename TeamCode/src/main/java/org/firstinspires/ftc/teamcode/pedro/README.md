## pedro autotune
<http://192.168.43.1:10158>

## ftc dash
<http://192.168.43.1:8080/dash>

## adb

mac
```bash
export ANDROID_HOME=$HOME/Library/Android/sdk export PATH=$PATH:$ANDROID_HOME/platform-tools
adb connect 192.168.43.1:5555
```

win
```
$env:ANDROID_HOME = "$env:LOCALAPPDATA\Android\Sdk"; $env:Path += ";$env:ANDROID_HOME\platform-tools"
adb connect 192.168.43.1:5555
```

```pwsh
[Environment]::SetEnvironmentVariable("Path", [Environment]::GetEnvironmentVariable("Path","User") + ";$env:LOCALAPPDATA\Android\Sdk\platform-tools", "User")
```