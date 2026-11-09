#!/usr/bin/env bash
# Install only patrol desktop integration; leaves networking and services alone.
set -euo pipefail
if (( EUID == 0 )); then
    echo '請以 yu 一般帳號執行，不要 sudo；避免裝到 root 的桌面。' >&2
    exit 1
fi
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
gui="$script_dir/patrol_debug_gui.py"
[[ -r "$gui" ]] || { echo "找不到 GUI：$gui" >&2; exit 1; }
/usr/bin/python3 -c 'import tkinter' || {
    echo '請先執行 sudo apt install python3-tk，再重新安裝。' >&2
    exit 1
}
/usr/bin/python3 "$gui" --install-autostart --desktop-shortcut "$@"
echo '已建立桌面捷徑與登入自動開啟設定。'
echo '若桌面要求信任捷徑，請右鍵選擇「允許啟動」。'
echo '未修改網路或服務，未啟動／停止車子。現在開啟可執行：'
printf 'python3 %q\n' "$gui"
