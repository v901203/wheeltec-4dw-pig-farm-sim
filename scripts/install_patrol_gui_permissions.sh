#!/usr/bin/env bash
# Run once on the Xavier host. This installer never starts/stops the robot.
set -euo pipefail
print_only=false
if [[ "${1:-}" == --print ]]; then
    print_only=true
    shift
fi
if (( $# > 1 )); then
    echo "用法：sudo bash $0 <帳號>；或 bash $0 --print <帳號>" >&2
    exit 2
fi
patrol_user="${1:-${SUDO_USER:-}}"
if [[ ! "$patrol_user" =~ ^[a-z_][a-z0-9_-]*$ ]] || [[ "$patrol_user" == root ]]; then
    echo '請指定一般使用者帳號，例如 yu。' >&2
    exit 2
fi
rule=$(printf '%s ALL=(root) NOPASSWD: /usr/bin/systemctl stop pig-pen-patrol.service, /usr/bin/systemctl restart pig-pen-patrol.service' "$patrol_user")
if "$print_only"; then
    printf '%s\n' "$rule"
    exit
fi
if (( EUID != 0 )); then
    echo '請使用 sudo 執行此一次性權限設定。' >&2
    exit 1
fi
id "$patrol_user" >/dev/null
[[ -x /usr/bin/systemctl && -x /usr/sbin/visudo ]]
# Abort before modifying anything if the existing sudo configuration is invalid.
/usr/sbin/visudo -c
install -d -o root -g root -m 0750 /etc/sudoers.d
rule_file="/etc/sudoers.d/pig-pen-gui-$patrol_user"
if [[ -e "$rule_file" || -L "$rule_file" ]]; then
    if [[ ! -L "$rule_file" ]] && [[ "$(cat "$rule_file")" == "$rule" ]]; then
        echo "免密碼規則已存在：$rule_file"
        exit
    fi
    echo "已有不同規則，請人工檢查，未覆寫：$rule_file" >&2
    exit 1
fi
# Dotted temporary filename is ignored by sudo's includedir loader.
tmp_rule=$(mktemp /etc/sudoers.d/.pig-pen-gui.XXXXXX)
trap 'rm -f "$tmp_rule"' EXIT
printf '%s\n' "$rule" > "$tmp_rule"
chown root:root "$tmp_rule"
chmod 0440 "$tmp_rule"
/usr/sbin/visudo -cf "$tmp_rule"
mv -T "$tmp_rule" "$rule_file"
if ! /usr/sbin/visudo -c; then
    rm -f "$rule_file"
    echo '規則整體驗證失敗，已撤銷本次新增規則。' >&2
    exit 1
fi
echo "已允許 $patrol_user 免密碼停止／重新啟動 pig-pen-patrol.service。"
echo '尚未執行任何車輛服務操作。請重新開啟新版 GUI。'
echo "撤銷方式：sudo rm '$rule_file'"
