# AGENTS.md

## Cursor Cloud specific instructions

### Overview
ECM (Enhanced Compression Model) is a C++17 video codec reference software — the successor to VVC/VTM. It contains an encoder (`EncoderApp`), decoder (`DecoderApp`), and several utility tools. There are no web services, databases, or external runtime dependencies.

### Build
The default system compiler on this VM is Clang, which fails to link due to a missing `-lstdc++` issue. **Always specify GCC explicitly** when invoking CMake:

```bash
cd /workspace/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
make -j$(nproc)
```

Two build directories are maintained:
- `build/` — Release (optimized, for benchmarking and testing)
- `build-debug/` — Debug (for development with symbols)

Binaries are placed under `bin/umake/gcc-13.3/x86_64/{release,debug}/`.

For convenience, the top-level `Makefile` wraps a Python build script (`cmake/CMakeBuild/bin/cmake.py`) and can build multiple variants at once: `make release`, `make debug`, `make all`. However, the direct CMake workflow above is more transparent and avoids the Python wrapper dependency.

### Testing
There are no automated unit/integration test suites. Correctness is verified by:
1. Encoding a test YUV sequence with `EncoderApp` using a config from `cfg/`.
2. Decoding the bitstream with `DecoderApp`.
3. Confirming the decoder output is bit-exact with the encoder's reconstruction (`-o` output).

Example quick smoke test:
```bash
# Generate a small test YUV (64x64, YUV420, 2 frames)
python3 -c "
w,h=64,64; data=bytearray()
for f in range(2):
    for y in range(h):
        for x in range(w): data.append((x*4+y*4+f*30)%256)
    for _ in range(2): 
        for y in range(h//2):
            for x in range(w//2): data.append(128)
open('/tmp/test.yuv','wb').write(data)
"
# Encode
bin/umake/gcc-13.3/x86_64/release/EncoderApp -c cfg/encoder_intra_ecm.cfg -i /tmp/test.yuv -b /tmp/out.bin -o /tmp/recon.yuv --SourceWidth=64 --SourceHeight=64 --FrameRate=30 --FramesToBeEncoded=2 --InputBitDepth=8 --QP=37
# Decode
bin/umake/gcc-13.3/x86_64/release/DecoderApp -b /tmp/out.bin -o /tmp/dec.yuv
# Verify
diff /tmp/recon.yuv /tmp/dec.yuv
```

### Lint / Static Analysis
No linter or static analysis configuration is included in the repository. The build uses `-Werror` (warnings as errors) which serves as a basic code quality gate.

### Key Gotchas
- The encoder config files in `cfg/` use `InternalBitDepth: 10` by default. When overriding `--InternalBitDepth=8`, the QP chroma tables become invalid. Only override `--InputBitDepth`, not `--InternalBitDepth`.
- The `TemporalSubsampleRatio` in intra configs is set to 8, so only every 8th frame is encoded. For a 2-frame input, only 1 frame will be encoded.
- The NNLF extension (`-DEXTENSION_NNLF=ON`) requires initializing the SADL git submodule first.
