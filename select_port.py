Import("env")
import os
import sys

try:
    import serial.tools.list_ports
except ImportError:
    serial = None

def get_target_names():
    targets = []
    # COMMAND_LINE_TARGETS から指定されたターゲット名を取得
    for t in BUILD_TARGETS:
        targets.append(str(t))
    for t in env.get("BUILD_TARGETS", []):
        targets.append(str(t))
    for t in getattr(sys, "argv", []):
        if t in ["upload", "monitor", "erase"]:
            targets.append(t)
    return set(targets)

def select_serial_port():
    # コンパイルのみ（ビルドのみ）の場合はポート選択をスキップ
    # upload, monitor, erase 等のポートを必要とするターゲットの時のみ実行
    action_targets = {"upload", "monitor", "erase", "run"}
    cli_targets = set(BUILD_TARGETS)
    
    # upload や monitor がターゲットに含まれているかチェック
    # （ターゲット指定がないビルド時は実行しない）
    is_upload = any("upload" in str(t).lower() for t in cli_targets)
    is_monitor = any("monitor" in str(t).lower() for t in cli_targets)
    is_erase = any("erase" in str(t).lower() for t in cli_targets)

    # sys.argv からも補助的に判定
    argv_str = " ".join(sys.argv)
    if "upload" in argv_str:
        is_upload = True
    if "monitor" in argv_str:
        is_monitor = True

    if not (is_upload or is_monitor or is_erase):
        return

    if not serial:
        return

    # ポート一覧の取得
    all_ports = list(serial.tools.list_ports.comports())
    if not all_ports:
        print("\n[Select Port] No serial ports found.\n")
        return

    # COM1 や ACPI 内蔵ポートを除外した候補リストを作成
    filtered_ports = []
    for p in all_ports:
        port_name = p.device.upper()
        hwid = (p.hwid or "").upper()
        desc = (p.description or "").upper()
        
        # COM1 や ACPI通信ポートは除外
        if port_name == "COM1" or "ACPI\\PNP0500" in hwid:
            continue
        filtered_ports.append(p)

    # もし除外した結果ゼロになった場合は全ポートを候補に戻す（安全策）
    candidates = filtered_ports if filtered_ports else all_ports

    # 候補が1つだけなら自動選択
    if len(candidates) == 1:
        chosen = candidates[0].device
        print(f"\n[Select Port] Automatically selected: {chosen} ({candidates[0].description})\n")
    else:
        # 複数ポートがある場合は対話形式で選択
        print("\n" + "=" * 50)
        print(" [Select Port] Multiple serial ports detected:")
        for idx, p in enumerate(candidates, 1):
            print(f"   [{idx}] {p.device} : {p.description}")
        print("=" * 50)

        # 最後に選択したポートのキャッシュを読み込む（あればデフォルトに）
        cache_file = os.path.join(env.subst("$PROJECT_DIR"), ".pio", ".last_selected_port")
        default_idx = 1
        if os.path.exists(cache_file):
            try:
                with open(cache_file, "r", encoding="utf-8") as f:
                    last_port = f.read().strip().upper()
                for idx, p in enumerate(candidates, 1):
                    if p.device.upper() == last_port:
                        default_idx = idx
                        break
            except Exception:
                pass

        prompt = f" Select port [1-{len(candidates)}] (default: {default_idx}): "
        chosen = None

        # 対話入力の受付
        while not chosen:
            try:
                user_input = input(prompt).strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[Select Port] Cancelled.")
                sys.exit(1)

            if not user_input:
                chosen = candidates[default_idx - 1].device
                break

            if user_input.isdigit():
                num = int(user_input)
                if 1 <= num <= len(candidates):
                    chosen = candidates[num - 1].device
                    break
            
            print(f" Invalid selection. Please enter a number between 1 and {len(candidates)}.")

        # 選択したポートをキャッシュに保存
        try:
            os.makedirs(os.path.dirname(cache_file), exist_ok=True)
            with open(cache_file, "w", encoding="utf-8") as f:
                f.write(chosen)
        except Exception:
            pass

        print(f"\n[Select Port] Selected: {chosen}\n")

    # PlatformIO の環境変数に反映
    if chosen:
        env.Replace(UPLOAD_PORT=chosen)
        env.Replace(MONITOR_PORT=chosen)

# 実行
select_serial_port()
