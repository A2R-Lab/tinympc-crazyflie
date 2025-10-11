# Neural Network Safety Filter Integration

## Summary

Integrated learned CBF (Control Barrier Function) from safe-reachability into Crazyflie firmware.

## Architecture

- **Network**: [12] → [1024] → [512] → [256] → [128] → [1]
- **Total Parameters**: ~700K
- **Quantization**: INT8 per-channel PTQ
- **Memory Footprint**: ~699 KB flash

## Implementation

### Files Created

1. **`safety_nn_weights.hpp`** - Float32 weights (unquantized, for reference)
2. **`safety_nn_int8_ptq.hpp`** - INT8 quantized weights with per-channel scales
3. **`safety_nn.hpp`** - Float32 inference engine (Eigen-based)
4. **`safety_nn_int8.hpp`** - INT8 inference engine (minimal, flash-resident weights)

### Optimizations Applied

1. **Float32 instead of Double** (2x savings)
   - Changed all `double` → `float` in SafetyNN
   - Uses M4F FPU efficiently

2. **LTO + Section GC** (10-15% code savings)
   - Added `-flto -ffunction-sections -fdata-sections` to `ARCH_CFLAGS`
   - Added `-flto` to `LDFLAGS` and section flags to `image_LDFLAGS`
   - Located in: `crazyflie-firmware/Makefile`

3. **INT8 Per-Channel Quantization** (4x weight savings)
   - Per output-channel weight scales
   - Calibrated activation scales (500 samples)
   - Runtime: INT8 matmul → FP32 output
   - Weights stay in flash (const)

### Export Scripts

Located in `safe-reachability/`:

- **`export_nn_to_cpp.py`** - Export float weights
- **`quantize_nn.py`** - Simple INT8 quantization
- **`export_nn_int8_ptq.py`** - Proper INT8 PTQ with calibration

## Memory Budget

### Current Firmware (before NN)
- Flash: 280 KB / 1,032 KB (27%)
- RAM: 85 KB / 131 KB (65%)

### With INT8 NN
- Flash: ~980 KB / 1,032 KB (95%) 
  - 280 KB firmware
  - 684 KB weights (INT8)
  - 15 KB scales + biases
  - ~50 KB free
- RAM: ~95 KB / 131 KB (72%)
  - 85 KB existing
  - ~10 KB activation buffers
  - ~36 KB free

**Status**: ⚠️ Tight but should fit!

## Next Steps

### Option 1: Build & Test (Current Network)
```bash
cd apps/controller_tinympc_eigen
make clean
make
```

Check flash/RAM usage in build output.

### Option 2: Further Compression (if needed)

**INT4 for Large Layers** (~350 KB total):
```python
# Quantize 1024×512 and 512×256 to INT4
# Keep others at INT8
# Expected: ~350 KB (half of 684 KB)
```

**Pruning + Sparsity**:
```python
# 70-90% zeros → CSR format
# Needs sparse GEMV kernel
```

### Option 3: Distill Smaller Network

Train: `[12] → [64] → [32] → [16] → [1]` (~3 KB)
- Knowledge distillation from big model
- Fast, minimal memory
- Retrain in `safe-reachability`

## Usage in Controller

```cpp
#include "safety_nn_int8.hpp"

SafetyNN_INT8::SafetyNNEvaluatorINT8 safety_nn;

// In control loop:
float state[12] = {x, y, z, vx, vy, vz, rp_x, rp_y, rp_z, wx, wy, wz};
float g_value = safety_nn.forward(state);

// Use g_value for CBF constraint
```

## Testing

1. **Compile Test**: Verify it builds and fits in flash
2. **Functional Test**: Compare outputs vs Python reference
3. **Performance Test**: Measure inference time (<2ms @ 500Hz)
4. **Flight Test**: Validate safety filtering in real flight

## Branch

All work is in: `nn-safety-filter` branch

