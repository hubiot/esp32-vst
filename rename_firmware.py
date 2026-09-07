Import("env")
import os
import shutil

def copy_named_firmware(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    firmware_bin = os.path.join(build_dir, "firmware.bin")
    if not os.path.exists(firmware_bin):
        return

    # FIRMWARE_VERSIONの定義値を取得
    fw_version = "firmware"
    defines = env.get("CPPDEFINES", [])
    for item in defines:
        if isinstance(item, tuple) and item[0] == "FIRMWARE_VERSION":
            fw_version = str(item[1]).replace('\\"', '').replace('"', '')
        elif isinstance(item, str) and item.startswith("FIRMWARE_VERSION="):
            fw_version = item.split("=")[1].replace('\\"', '').replace('"', '')

    # プロジェクト直下の bin フォルダに出力
    out_dir = os.path.join(env.subst("$PROJECT_DIR"), "bin")
    os.makedirs(out_dir, exist_ok=True)

    out_name = f"{fw_version}.bin"
    target_path = os.path.join(out_dir, out_name)
    shutil.copyfile(firmware_bin, target_path)

    # ビルドフォルダ内にも同じ名前でコピー
    shutil.copyfile(firmware_bin, os.path.join(build_dir, out_name))

    print(f"\n==================================================")
    print(f" [Firmware Binary Created]")
    print(f"  -> {target_path}")
    print(f"==================================================\n")

# firmware.bin 作成完了後に実行
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_named_firmware)
