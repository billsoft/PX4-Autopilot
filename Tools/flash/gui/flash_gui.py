import os
import sys
import shutil
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

def find_cli(cli_arg):
    if cli_arg and os.path.isfile(cli_arg):
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
    if file_arg and os.path.isfile(file_arg):
        return file_arg
    if not target:
        return None
    build_dir = os.path.join(os.getcwd(), "build", target)
    fname = f"{target}.{image_type}"
    path = os.path.join(build_dir, fname)
    return path if os.path.isfile(path) else None

class FlashGUI:
    def __init__(self, root):
        self.root = root
        root.title("PX4 固件烧写 GUI")
        frm = ttk.Frame(root, padding=8)
        frm.grid(sticky="nsew")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)

        self.target = tk.StringVar(value="st_nucleo-h743zi-fc_default")
        self.image = tk.StringVar(value="bin")
        self.file = tk.StringVar()
        self.address = tk.StringVar(value="0x08000000")
        self.erase = tk.StringVar(value="all")
        self.cli = tk.StringVar(value=find_cli(None) or "")
        self.status = tk.StringVar(value="待机")

        ttk.Label(frm, text="目标").grid(row=0, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.target, width=32).grid(row=0, column=1, sticky="ew")

        ttk.Label(frm, text="镜像类型").grid(row=0, column=2, sticky="e")
        ttk.OptionMenu(frm, self.image, "bin", "bin", "elf", "px4", command=self.on_image_change).grid(row=0, column=3, sticky="ew")

        ttk.Label(frm, text="固件文件").grid(row=1, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.file, width=48).grid(row=1, column=1, columnspan=2, sticky="ew")
        ttk.Button(frm, text="浏览", command=self.browse_file).grid(row=1, column=3, sticky="ew")

        ttk.Label(frm, text="地址").grid(row=2, column=0, sticky="w")
        self.addr_entry = ttk.Entry(frm, textvariable=self.address, width=32)
        self.addr_entry.grid(row=2, column=1, sticky="ew")

        ttk.Label(frm, text="擦除").grid(row=2, column=2, sticky="e")
        ttk.OptionMenu(frm, self.erase, "all", "all", "none").grid(row=2, column=3, sticky="ew")

        ttk.Label(frm, text="CLI路径").grid(row=3, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.cli, width=48).grid(row=3, column=1, columnspan=2, sticky="ew")
        ttk.Button(frm, text="浏览", command=self.browse_cli).grid(row=3, column=3, sticky="ew")

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=4, sticky="ew", pady=6)
        ttk.Button(btns, text="自动探测固件", command=self.autodetect_file).grid(row=0, column=0, padx=4)
        ttk.Button(btns, text="检测环境", command=self.check_env).grid(row=0, column=1, padx=4)
        ttk.Button(btns, text="开始烧写", command=self.start_flash).grid(row=0, column=2, padx=4)
        ttk.Button(btns, text="打开构建目录", command=self.open_build_dir).grid(row=0, column=3, padx=4)
        ttk.Button(btns, text="从ELF导出BIN", command=self.objcopy_bin).grid(row=0, column=4, padx=4)

        self.log = tk.Text(frm, height=20)
        self.log.grid(row=5, column=0, columnspan=4, sticky="nsew")
        frm.rowconfigure(5, weight=1)
        frm.columnconfigure(1, weight=1)

        ttk.Label(frm, textvariable=self.status).grid(row=6, column=0, columnspan=4, sticky="w")
        self.on_image_change(self.image.get())

    def on_image_change(self, val):
        state = "normal" if val == "bin" else "disabled"
        self.addr_entry.configure(state=state)

    def browse_file(self):
        types = [("BIN", "*.bin"), ("ELF", "*.elf"), ("PX4", "*.px4"), ("All", "*.*")]
        f = filedialog.askopenfilename(filetypes=types)
        if f:
            self.file.set(f)

    def browse_cli(self):
        f = filedialog.askopenfilename()
        if f:
            self.cli.set(f)

    def autodetect_file(self):
        p = resolve_image(self.target.get(), self.image.get(), None)
        if p:
            self.file.set(p)
            messagebox.showinfo("完成", "已探测到固件")
        else:
            messagebox.showwarning("未找到", "未在构建目录找到固件，请手动选择或先构建")

    def check_env(self):
        cli = find_cli(self.cli.get())
        if not cli:
            messagebox.showerror("错误", "未找到 STM32CubeProgrammer CLI")
            return
        if not self.file.get() or not os.path.isfile(self.file.get()):
            messagebox.showerror("错误", "固件文件不存在")
            return
        self.status.set("环境就绪")
        messagebox.showinfo("完成", "环境检测通过")

    def start_flash(self):
        cli = find_cli(self.cli.get())
        if not cli:
            messagebox.showerror("错误", "未找到 CLI")
            return
        img = self.file.get()
        if not img:
            img = resolve_image(self.target.get(), self.image.get(), None)
        if not img or not os.path.isfile(img):
            messagebox.showerror("错误", "固件文件不存在")
            return
        cmd = [cli, "-c", "port=SWD"]
        if self.erase.get() == "all":
            cmd += ["-e", "all"]
        if self.image.get() == "bin":
            cmd += ["-w", img, self.address.get()]
        else:
            cmd += ["-w", img]
        cmd += ["-v", "-rst"]
        self.log.delete("1.0", tk.END)
        self.status.set("烧写中")
        t = threading.Thread(target=self.run_cmd, args=(cmd,), daemon=True)
        t.start()

    def run_cmd(self, cmd):
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("错误", str(e)))
            self.root.after(0, lambda: self.status.set("失败"))
            return
        for line in p.stdout:
            self.root.after(0, lambda s=line: self.log.insert(tk.END, s))
        rc = p.wait()
        self.root.after(0, lambda: self.status.set("成功" if rc == 0 else f"失败({rc})"))

    def open_build_dir(self):
        d = os.path.join(os.getcwd(), "build", self.target.get())
        if os.path.isdir(d):
            try:
                os.startfile(d)
            except Exception:
                messagebox.showinfo("提示", d)
        else:
            messagebox.showwarning("未找到", "构建目录不存在")

    def objcopy_bin(self):
        elf = resolve_image(self.target.get(), "elf", None)
        if not elf:
            messagebox.showwarning("未找到", "未找到 ELF")
            return
        out = os.path.join(os.path.dirname(elf), f"{self.target.get()}.bin")
        tool = shutil.which("arm-none-eabi-objcopy")
        if not tool:
            messagebox.showwarning("未找到", "未找到 arm-none-eabi-objcopy")
            return
        cmd = [tool, "-O", "binary", elf, out]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True)
        except Exception as e:
            messagebox.showerror("错误", str(e))
            return
        if r.returncode == 0 and os.path.isfile(out):
            self.file.set(out)
            messagebox.showinfo("完成", "已生成 BIN")
        else:
            messagebox.showerror("错误", r.stderr or r.stdout or "生成失败")

def main():
    root = tk.Tk()
    FlashGUI(root)
    root.mainloop()

if __name__ == "__main__":
    sys.exit(main())

