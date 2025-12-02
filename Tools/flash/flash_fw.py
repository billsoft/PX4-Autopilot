import argparse
import os
import shutil
import subprocess
import sys

def find_cli(cli_arg):
    if cli_arg:
        return cli_arg
    env_cli = os.environ.get("STM32_CLI")
    if env_cli and os.path.isfile(env_cli):
        return env_cli
    default_cli = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    if os.path.isfile(default_cli):
        return default_cli
    which_cli = shutil.which("STM32_Programmer_CLI")
    if which_cli:
        return which_cli
    return None

def resolve_image(target, image_type, file_arg):
    if file_arg:
        return file_arg
    if not target:
        return None
    build_dir = os.path.join("build", target)
    if image_type == "bin":
        f = os.path.join(build_dir, f"{target}.bin")
    elif image_type == "elf":
        f = os.path.join(build_dir, f"{target}.elf")
    else:
        f = os.path.join(build_dir, f"{target}.px4")
    return f if os.path.isfile(f) else None

def sanitize_line(s: str) -> str:
    out = []
    for ch in s:
        if ord(ch) < 128:
            out.append(ch)
        else:
            out.append('#')
    return ''.join(out)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--target", default="st_nucleo-h743zi-fc_default")
    p.add_argument("--image", choices=["bin","elf","px4"], default="bin")
    p.add_argument("--file", default=None)
    p.add_argument("--address", default="0x08000000")
    p.add_argument("--erase", choices=["none","all"], default="all")
    p.add_argument("--cli", default=None)
    args = p.parse_args()

    cli = find_cli(args.cli)
    if not cli:
        print("STM32CubeProgrammer CLI 未找到。请安装后设置 STM32_CLI 环境变量或使用 --cli 指定路径。")
        sys.exit(1)

    image = resolve_image(args.target, args.image, args.file)
    if not image:
        print("固件文件不存在。请使用 --file 指定或先构建生成 .bin/.elf/.px4。")
        sys.exit(1)

    cmd = [cli, "-c", "port=SWD"]
    if args.erase == "all":
        cmd += ["-e", "all"]
    if args.image == "bin":
        cmd += ["-w", image, args.address]
    else:
        cmd += ["-w", image]
    cmd += ["-v", "-rst"]

    print("执行:", " ".join(cmd))
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    except Exception as e:
        print(str(e))
        sys.exit(1)
    rc = 0
    if p.stdout is not None:
        for line in p.stdout:
            print(sanitize_line(line), end='')
    rc = p.wait()
    sys.exit(rc)

if __name__ == "__main__":
    main()

