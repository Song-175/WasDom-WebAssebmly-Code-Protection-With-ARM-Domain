# WasDom-WebAssembly-Code-Protection-With-ARM-Domain

This repository contains **only the `src` folder** from the Chrome V8 engine.  
To use it in the WasDom environment, you must first clone the official V8 repository,  
then **replace the `src` folder** in that cloned repository with the one from this repository.

---

## Requirements

- **Raspberry Pi OS (32-bit)** installed on an RPi device  
- **Network and SSH** configured for remote shell access  
- In `FAT32/config.txt`, add:
  ```bash
  kernel=kernel7l.img
  ```

---

## 1. Building V8 (ARM)

Below is an example of building V8 for ARM (e.g., Raspberry Pi).  
Refer to the official V8 documentation for details.

### Clone and Set Up `depot_tools`, Then Fetch V8

```bash
# Install depot_tools
git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git
export PATH=/path/to/depot_tools:$PATH

# Prepare V8 folder
mkdir ~/v8
cd ~/v8
fetch v8
cd v8
```

### 2. Build V8

```bash
gn gen out/arm.debug --args='is_debug=true target_cpu="arm" v8_target_cpu="arm" is_component_build=false'
ninja -C out/arm.debug d8
```

*(Refer to [Build V8 with GN](https://v8.dev/docs/build-gn))*

### 3. Copy the Built Binaries to the Raspberry Pi

```bash
scp ./out/arm.debug/d8 user@192.168.1.161:/home/user/
scp ./out/arm.debug/icudtl.dat user@192.168.1.161:/home/user/
scp ./out/arm.debug/snapshot_blob.bin user@192.168.1.161:/home/user/
```

### 4. (Optional) Transfer Example Test Files

```bash
scp -r ./test/wasm-spec-tests/tests user@192.168.1.161:/home/user/tests
```

For instance, you can use `.js` files under `test/wasm-spec-tests/`.

### 5. Run `d8` on the Raspberry Pi

```bash
./d8 --print-wasm-code tests/call_indirect.js > dump
```

---

## *EXTRA* - Modifying and Rebuilding V8

```bash
# Create a build directory
gn gen out/arm.domain0208 --args='is_debug=true target_cpu="arm" v8_target_cpu="arm" is_component_build=false'
cd out/arm.domain0208

# Edit toolchain.ninja (lines 79-87) in vim or similar
vi toolchain.ninja

# Rebuild
ninja -C out/arm.domain0208 d8
```
