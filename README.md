# BFV Homomorphic Encryption: FPGA-Based Secure Image Transfer On the cloud using BFV Homomorphic Algorithm

**Hardware-accelerated Brakerski/Fan-Vercauteren (BFV) scheme with NTT-accelerated polynomial multiplication**

[Introduction](#introduction) • [Architecture](#architecture) • [Math Framework](#mathematical-framework) • [NTT Implementation](#ntt-implementation) • [Setup](#getting-started)

---

## 🚧 Project Status

**Current Phase**: Active Development

This project is currently under development. Core modules are being implemented and tested incrementally. Documentation will be updated as features are completed and validated.

---

## 📊 Technology Implementation Summary

| **Aspect** | **Specification** | **Details** |
|---|---|---|
| **Target Platform** | Zynq-7000 FPGA | Initial hardware validation node |
| **Arithmetic Width** | 30-Bit | Optimized modular arithmetic pipeline |
| **Logic Type** | Ring-LWE with NTT | Full polynomial ring implementation |
| **Polynomial Degree (n)** | 256/512/1024 | Configurable security parameter |
| **NTT Radix** | Radix-2 DIT/DIF | Decimation-in-time/frequency |
| **Secret Key (K)** | Dynamic Generation | LFSR-based polynomial coefficients |
| **Throughput** | 4096 Pixels | Processes standard test image RAM |
| **I/O Interface** | UART | Secure encrypted serial transfer |

---

## 📌 Introduction

This repository presents the **Design and Implementation of FPGA-Based BFV Homomorphic Encryption with NTT Acceleration**. The BFV scheme is a somewhat homomorphic encryption method based on the **Ring Learning With Errors (RLWE)** problem, enhanced with Number Theoretic Transform (NTT) for efficient polynomial multiplication in hardware.

### 🎯 Project Highlights

- **NTT/INTT Cores**: Hardware-accelerated polynomial multiplication using Number Theoretic Transform
- **Ring-LWE Implementation**: Full polynomial ring arithmetic R_q = Z_q[x]/(x^n + 1)
- **Modular PE**: Configurable Processing Element for dynamic Add/Sub/Mult/NTT selection
- **30-Bit Pipeline**: High-precision low-level modular arithmetic units
- **Twiddle Factor ROM**: Pre-computed primitive roots of unity for NTT operations
- **Secure Transfer**: Integrated UART transmitter for "FPGA-to-Cloud" data flow

### 🔄 Development Roadmap

- [x] Basic modular arithmetic units (Add, Sub, Mult)
- [x] Initial BFV encryption core with scalar operations
- [ ] NTT butterfly unit implementation
- [ ] Twiddle factor ROM generation
- [ ] Complete NTT/INTT pipeline
- [ ] Polynomial multiplication via NTT
- [ ] Full BFV encryption with Ring-LWE
- [ ] Hardware validation and timing analysis
- [ ] Performance benchmarking

---

## 🏗️ Architecture

### System Block Diagram with NTT Integration

The system is designed with NTT acceleration modules integrated into the encryption pipeline.
```
┌─────────────────────────────────────────────────────────────────┐
│      FPGA-BASED BFV ENCRYPTION WITH NTT ARCHITECTURE            │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────┐      ┌───────────────────────────┐
│   IMAGE BRAM     │      │   TOP LEVEL CONTROLLER    │
│ (4096 x 8-bit)   │◄─────┤      (7-State FSM)        │
└────────┬─────────┘      └─────────────┬─────────────┘
         │                              │
┌────────▼─────────┐      ┌─────────────▼─────────────┐
│ POLYNOMIAL BUFFER│      │   NTT CONTROL LOGIC       │
│  (Coefficient    │◄─────┤  (Forward/Inverse Select) │
│   Storage RAM)   │      └─────────────┬─────────────┘
└────────┬─────────┘                    │
         │                ┌─────────────▼─────────────┐
         │                │    TWIDDLE FACTOR ROM     │
         │                │  (Pre-computed ω values)  │
         │                └─────────────┬─────────────┘
         │                              │
┌────────▼──────────────────────────────▼─────────────┐
│              NTT/INTT BUTTERFLY UNITS                │
│      (Cooley-Tukey Radix-2 DIT/DIF Pipeline)        │
└────────┬─────────────────────────────────────────────┘
         │
┌────────▼─────────┐      ┌─────────────────────────────┐
│ ENCRYPTION CORE  │      │   CONFIGURABLE PE (ALU)     │
│  (BFV_Core.v)    │◄─────┤ (Add/Sub/Mult/PolyMult)     │
└────────┬─────────┘      └─────────────┬───────────────┘
         │                              │
┌────────▼─────────┐      ┌─────────────▼───────────────┐
│  UART TX DRIVER  │      │   LOW-LEVEL MATH UNITS      │
│ (10416 baud rate)│      │ (Modular_Adder/Sub/Mult)    │
└──────────────────┘      └─────────────────────────────┘
```

---

## 📚 Mathematical Framework

The scheme operates over the polynomial ring **R_q = Z_q[x]/(x^n + 1)**.

### Parameters

- **n**: Degree of the polynomial (power of 2: 256, 512, 1024)
- **q**: Ciphertext modulus (NTT-friendly prime: q ≡ 1 mod 2n)
- **t**: Plaintext modulus (t << q)
- **Δ**: Scaling factor, Δ = ⌊q/t⌋
- **ω**: Primitive 2n-th root of unity mod q

### BFV Encryption Operations

**Key Generation:**
- Secret key: s(x) ∈ R_q (small coefficients from discrete Gaussian)
- Public key: (pk₀, pk₁) where pk₀ = -(a·s + e) mod q, pk₁ = a

**Encryption:**
```
m(x) → plaintext polynomial
ct₀ = pk₀·u + e₁ + Δ·m(x) mod q
ct₁ = pk₁·u + e₂ mod q
Ciphertext = (ct₀, ct₁)
```

**Polynomial Multiplication via NTT:**
```
c(x) = a(x) · b(x) mod (x^n + 1)

Standard: O(n²) coefficient-wise multiplication
NTT-accelerated: O(n log n)

Steps:
1. â = NTT(a)     // Forward transform
2. b̂ = NTT(b)     // Forward transform
3. ĉ = â ⊙ b̂      // Point-wise multiplication
4. c = INTT(ĉ)    // Inverse transform
```

---

## 🔧 NTT Implementation

### Number Theoretic Transform (NTT)

The NTT is the discrete Fourier transform over finite fields, enabling fast polynomial multiplication.

**Forward NTT:**
```
â[k] = Σ(i=0 to n-1) a[i] · ω^(i·k) mod q
where ω is a primitive 2n-th root of unity
```

**Inverse NTT:**
```
a[i] = n⁻¹ · Σ(k=0 to n-1) â[k] · ω^(-i·k) mod q
```

### Hardware NTT Architecture

**1. Twiddle Factor ROM (`Twiddle_ROM.v`)**
```verilog
// Pre-computed powers of ω for n=1024
module Twiddle_ROM #(
    parameter N = 1024,
    parameter WIDTH = 30
)(
    input wire [9:0] address,
    output reg [WIDTH-1:0] twiddle_factor
);
    // ROM stores ω^0, ω^1, ω^2, ..., ω^(n-1) mod q
endmodule
```

**2. NTT Butterfly Unit (`NTT_Butterfly.v`)**
```verilog
// Cooley-Tukey Radix-2 Butterfly
module NTT_Butterfly #(
    parameter WIDTH = 30
)(
    input wire [WIDTH-1:0] x_in,
    input wire [WIDTH-1:0] y_in,
    input wire [WIDTH-1:0] twiddle,
    output wire [WIDTH-1:0] x_out,
    output wire [WIDTH-1:0] y_out
);
    // x_out = x_in + (y_in * twiddle) mod q
    // y_out = x_in - (y_in * twiddle) mod q
endmodule
```

**3. NTT Core (`NTT_Core.v`)**
```verilog
module NTT_Core #(
    parameter N = 1024,
    parameter WIDTH = 30
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire mode,  // 0: Forward NTT, 1: Inverse NTT
    input wire [WIDTH-1:0] coeff_in,
    output reg [WIDTH-1:0] coeff_out,
    output reg done
);
    // log2(N) = 10 stages for N=1024
    // Each stage processes N/2 butterflies
endmodule
```

### NTT-Based Polynomial Multiplier

**Integration Module (`Poly_Mult_NTT.v`)**
```verilog
module Poly_Mult_NTT #(
    parameter N = 1024,
    parameter WIDTH = 30
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [WIDTH-1:0] poly_a [0:N-1],
    input wire [WIDTH-1:0] poly_b [0:N-1],
    output reg [WIDTH-1:0] poly_c [0:N-1],
    output reg done
);
    // State machine:
    // 1. NTT(a) → â
    // 2. NTT(b) → b̂
    // 3. Point-wise: ĉ[i] = â[i] * b̂[i] mod q
    // 4. INTT(ĉ) → c
endmodule
```

---

## 🛠️ Practical Implementation

### 1. Math Modules (`Math_Modules.v`)

Enhanced atomic Verilog units:

- **Modular_Adder**: (A + B) mod q
- **Modular_Subtractor**: (A - B) mod q  
- **Modular_Multiplier**: (A × B) mod q with Barrett reduction
- **Modular_Inverse**: Computes n⁻¹ mod q for INTT scaling

### 2. Processing Element (`Configurable_PE.v`)

Extended ALU with NTT support via 3-bit opcodes:

- `000`: Addition
- `001`: Subtraction
- `010`: Multiplication
- `011`: Polynomial Multiplication (NTT-accelerated)
- `100`: NTT Forward Transform
- `101`: NTT Inverse Transform

### 3. Top Level FSM (`FPGA_Cloud_Top.v`)

| **State** | **Value** | **Function** |
|---|---|---|
| **s_IDLE** | `3'b000` | Wait for i_Start signal |
| **s_LOAD_POLY** | `3'b001` | Load polynomial coefficients from BRAM |
| **s_NTT_FORWARD** | `3'b010` | Perform forward NTT on input polynomials |
| **s_POINTWISE_MULT** | `3'b011` | Multiply transformed coefficients |
| **s_NTT_INVERSE** | `3'b100` | Perform inverse NTT to recover result |
| **s_ENCRYPT** | `3'b101` | Apply BFV encryption |
| **s_SEND** | `3'b110` | Start UART transmission |
| **s_WAIT_TX** | `3'b111` | Wait for Tx_Done flag |

### 4. NTT Parameter Selection

**NTT-Friendly Prime Selection:**
```
For n = 1024:
q must satisfy q ≡ 1 (mod 2048)

Example primes:
- q = 132120577 (27-bit, ω = 12277)
- q = 268435457 (29-bit, ω = 5)
- q = 1073872897 (30-bit, ω = 11)
```

**Primitive Root Verification:**
```python
# Python script to verify ω is primitive 2n-th root
def is_primitive_root(omega, n, q):
    return pow(omega, n, q) == q - 1  # ω^n ≡ -1 (mod q)
    and pow(omega, 2*n, q) == 1       # ω^(2n) ≡ 1 (mod q)
```

---

## 🚀 Getting Started

### Prerequisites

- **Target Hardware**: Xilinx Zynq-7000 FPGA (minimum 100k LUTs for n=1024)
- **Toolchain**: Vivado Design Suite 2020.1+
- **Language**: Verilog HDL
- **Memory**: 4MB BRAM (for polynomial storage and twiddle factors)

### Installation

1. **Clone the Repository**
```bash
git clone https://github.com/divyansh404-sudo/fpga-bfv-encryption.git
cd fpga-bfv-encryption
```

2. **Setup Vivado Project**

Add the following modules to your project:
- `Math_Modules.v` - Base modular arithmetic
- `Twiddle_ROM.v` - NTT twiddle factors
- `NTT_Butterfly.v` - Butterfly computation unit
- `NTT_Core.v` - Complete NTT/INTT pipeline
- `Poly_Mult_NTT.v` - NTT-based polynomial multiplier
- `Configurable_PE.v` - Enhanced processing element
- `BFV_Encryption_Core.v` - Main encryption logic
- `FPGA_Cloud_Top.v` - Top-level controller

3. **Generate Twiddle Factors**
```python
# Python script: generate_twiddle.py
import math

def generate_twiddle_rom(n, q, omega):
    """Generate Verilog ROM initialization for twiddle factors"""
    twiddles = [pow(omega, i, q) for i in range(n)]
    
    with open("twiddle_rom_init.v", "w") as f:
        f.write(f"// Twiddle factors for n={n}, q={q}\n")
        for i, tw in enumerate(twiddles):
            f.write(f"    twiddle_mem[{i}] = 30'd{tw};\n")

# Example: n=1024, q=1073872897, ω=11
generate_twiddle_rom(1024, 1073872897, 11)
```

4. **Synthesize and Implement**
```tcl
# Vivado TCL commands
read_verilog [ glob *.v ]
synth_design -top FPGA_Cloud_Top -part xc7z020clg484-1
opt_design
place_design
route_design
write_bitstream -force top.bit
```

---

## 📈 Expected Performance Analysis

### Projected Resource Utilization (Zynq-7020)

| **Module** | **LUTs** | **FFs** | **DSPs** | **BRAM** |
|---|---|---|---|---|
| Modular Multiplier (30-bit) | ~1,245 | ~890 | 4 | 0 |
| NTT Butterfly Unit | ~2,380 | ~1,450 | 8 | 0 |
| NTT Core (n=1024) | ~45,200 | ~32,768 | 128 | 16 |
| Twiddle ROM | ~512 | 0 | 0 | 4 |
| BFV Encryption Core | ~8,940 | ~6,200 | 16 | 8 |
| **Total System (Estimated)** | **~68,500** | **~52,100** | **~180** | **~32** |

### Target Timing Performance

| **Operation** | **Target Latency** | **Expected Throughput** |
|---|---|---|
| 30-bit Modular Multiplication | 2 cycles | 500 MHz |
| NTT (n=1024) | 10,240 cycles | ~97.7 μs @ 100 MHz |
| INTT (n=1024) | 10,240 cycles | ~97.7 μs @ 100 MHz |
| Polynomial Multiplication | 25,600 cycles | ~256 μs @ 100 MHz |
| Full BFV Encryption | 58,000 cycles | ~580 μs @ 100 MHz |

### Theoretical Comparison: NTT vs. Standard Multiplication

| **Method** | **Complexity** | **Expected Time (n=1024)** | **Speedup** |
|---|---|---|---|
| Standard School Multiplication | O(n²) | ~104 ms | 1× |
| Karatsuba | O(n^1.585) | ~18 ms | 5.8× |
| **NTT (Hardware)** | **O(n log n)** | **~0.256 ms** | **~406×** |

*Note: Performance figures are theoretical projections based on design specifications and will be validated during hardware testing.*

---

## 🔮 Future Scope

### Enhanced NTT Optimizations

- **Radix-4 Butterflies**: Reduce NTT stages from log₂(n) to log₄(n) for 40% latency reduction
- **Parallel Butterfly Units**: Deploy 4-8 butterfly units in parallel for 4-8× throughput increase
- **Constant-Geometry NTT**: Eliminate bit-reversal permutation overhead
- **Mixed-Radix NTT**: Support non-power-of-2 polynomial degrees (e.g., n=768)

### Advanced Cryptographic Features

- **RNS (Residue Number System)**: Multi-modulus representation for larger ciphertext modulus
- **Bootstrapping Support**: Implement key-switching and modulus-switching for unbounded computation depth
- **CKKS Variant**: Extend to approximate arithmetic for encrypted machine learning
- **Multi-Party Computation**: Threshold encryption with distributed key generation

### System-Level Integration

- **PCIe Interface**: Replace UART with PCIe Gen3 x8 for 64 Gbps encrypted data transfer
- **AES-GCM Hybrid**: Combine FHE with symmetric encryption for practical cloud deployment
- **OpenFHE Integration**: Hardware acceleration backend for open-source FHE library
- **AWS F1 Deployment**: Cloud-scale encrypted computation on Amazon FPGA instances

### Security Hardening

- **Side-Channel Resistance**: 
  - Constant-time modular reduction (Barrett/Montgomery)
  - Masked NTT butterflies resistant to DPA (Differential Power Analysis)
  - Random delay insertion in FSM transitions
- **Fault Injection Protection**: Redundant computation with error detection
- **Key Zeroization**: Secure key erasure on tamper detection

---

## 🎯 Project Goals

This ongoing project aims to demonstrate a complete hardware implementation of BFV homomorphic encryption with NTT acceleration on FPGA. The target objectives include:

### Core Objectives

- **NTT Integration**: Hardware-optimized Cooley-Tukey radix-2 pipeline with pre-computed twiddle factors
- **Polynomial Ring Arithmetic**: Full R_q = Z_q[x]/(x^n + 1) implementation with configurable security parameters
- **Resource Efficiency**: Complete n=1024 BFV system designed to fit in Zynq-7020 with <70% LUT utilization
- **Modular Architecture**: Clean separation enabling independent optimization of NTT, arithmetic, and control layers

### Impact and Significance

The convergence of homomorphic encryption and FPGA acceleration addresses the critical performance gap preventing FHE adoption in production systems. While software FHE libraries achieve ~1000× slowdown compared to plaintext computation, hardware acceleration targets reduction to ~10-50×, making real-world encrypted cloud computing more feasible.

This implementation will serve as both an educational reference for Ring-LWE cryptography and a practical foundation for building encrypted computation accelerators. The NTT-optimized design patterns will generalize beyond BFV to other lattice-based schemes (BGV, CKKS, TFHE) and post-quantum cryptographic primitives.

As quantum computers threaten classical public-key systems, lattice-based encryption becomes essential for long-term security. Hardware implementations like this project will contribute to tomorrow's quantum-resistant cryptographic infrastructure.

---

## 📚 References

### Core Papers

1. **BFV Scheme**: Brakerski, Z. (2012). "Fully Homomorphic Encryption without Modulus Switching from Classical GapSVP"
2. **NTT Optimization**: Longa, P., & Naehrig, M. (2016). "Speeding up the Number Theoretic Transform for Faster Ideal Lattice-Based Cryptography"
3. **FPGA Implementation**: Roy, S. S., et al. (2014). "Compact Ring-LWE Cryptoprocessor"

### Implementation Resources

- **OpenFHE**: https://github.com/openfheorg/openfhe-development
- **SEAL Library**: https://github.com/microsoft/SEAL
- **NTT Tutorial**: https://www.nayuki.io/page/number-theoretic-transform-integer-dft

---

## 📧 Contact

**Divyansh Tiwari**

- 📬 Email: divyanshtiwari435@gmail.com
- 💼 LinkedIn: [Divyansh Tiwari](https://www.linkedin.com/in/divyansh-tiwari-18064728a)
- 🐙 GitHub: [divyansh404-sudo](https://github.com/divyansh404-sudo)

---

© 2026 Divyansh Tiwari. All Rights Reserved.
