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

# Reference
- https://www.m3tech.blog/entry/2023/03/03/180619
