# rv32-zig

```bash
$ git clone --recursive https://github.com/naoki9911/rv32-zig
$ cd rv32-zig
$ docker build -f Dockerfile.dev .
```

In the docker image,
```bash
$ cd riscv-tests/isa
$ make -j $(nproc) && find . -maxdepth 1 -perm -111 -type f | xargs -I {} riscv64-unknown-elf-objcopy -O binary {} {}.bin
```

# Specification
- https://drive.google.com/file/d/1uviu1nH-tScFfgrovvFCrj7Omv8tFtkp/view?usp=drive_link
- Version: 20240411
- Published May 2024

# Reference
- https://www.m3tech.blog/entry/2023/03/03/180619
- https://github.com/takahirox/riscv-rust?tab=readme-ov-file
- https://bokuweb.github.io/undefined/articles/20230523.html
