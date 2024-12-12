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
- https://blog.rogiken.org/blog/2023/04/06/32bit-risc-v-linux%E3%82%92%E4%BD%9C%E3%82%8Aqemu%E3%81%A7%E5%AE%9F%E8%A1%8C%E3%81%99%E3%82%8B/
- https://diary.shift-js.info/building-a-riscv-cpu-for-linux/
