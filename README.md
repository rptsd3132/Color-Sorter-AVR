# 🌈 Chroma-Sifter: Automated Ball Sorting System

Welcome to the **Chroma-Sifter** — an autonomous electro‑mechanical system designed to detect and sort colorful spheres using the **ATmega328P** microcontroller. This project automates the entire sorting workflow using color sensing, servo actuation, and efficient embedded firmware.

---

## 📘 Overview

The Chroma-Sifter identifies ball colors using dual **TCS3200 color sensors** and redirects them into appropriate bins using servo-driven gates. Everything is controlled via optimized firmware running on the ATmega328P.

---

## 🧩 Feature Summary

| Feature             | Description                     |
| ------------------- | ------------------------------- |
| **Microcontroller** | ATmega328P                      |
| **Clock Speed**     | 16 MHz                          |
| **Color Sensors**   | Dual TCS3200 modules            |
| **Actuators**       | 2× Servos (Lift + Sorting Gate) |
| **Programmer**      | USBasp (via USB)                |
| **Build System**    | WinAVR Makefile Template        |
| **Output Format**   | Intel HEX (ihex)                |
| **Debug Format**    | DWARF‑2                         |

---

## 🔬 How It Works

### 1. **Color Detection Algorithm**

The TCS3200 sensors output frequency signals proportional to RGB intensities.

To determine color, the system calculates normalized ratios:

* **R/G**
* **B/G**

These values are compared against calibrated thresholds:

| Color      | R/G Range  | B/G Range   |
| ---------- | ---------- | ----------- |
| **Red**    | 1.9 – 2.6  | 1.28 – 1.69 |
| **Yellow** | 1.3 – 1.5  | 0.65 – 0.8  |
| **Green**  | 0.82 – 1.1 | 0.9 – 1.12  |

The function **`distanceFromThreshold()`** computes deviation from ideal bounds. The color with the smallest deviation is selected.

---

### 2. **Sorting Cycle**

A full cycle repeats every **20 seconds**.

1. **Lift (0° → 90°)** – First servo raises the ball for sensing.
2. **Sense (90°)** – Dual TCS3200 sensors sample frequency values.
3. **Drop (90° → 180° → 0°)** – Ball is released after sensing.
4. **Sort** – Gate servo moves according to detected color.

| Color      | Gate Servo Angle |
| ---------- | ---------------- |
| **Green**  | 10°              |
| **Red**    | 65°              |
| **Yellow** | 120°             |

---

## 🛠️ Build & Upload Instructions

### 🔧 Prerequisites

Make sure you have:

* **AVR-GCC toolchain**
* **AVRDUDE**
* **USBasp** programmer

---

### 1. **Compile the Firmware**

```bash
make all
```

This compiles `main.c` using size optimization.

---

### 2. **Upload to ATmega328P**

```bash
sudo make program
```

> ⚠️ If you're not using Linux or don't want sudo, edit the Makefile accordingly.

---

### 3. **Debugging (Optional)**

Generate COFF:

```bash
make coff
```

Start debugging with `avarice` + `insight`:

```bash
make debug
```

---

### 4. **Clean Build Files**

```bash
make clean
```

---

## 📸 System Showcase

* Physical sorter prototype
* TCS3200 modules
* Color sensing chamber

*(Add your images here once uploaded to the repository.)*

---

## 📄 License

Include your preferred license here (MIT, GPL, etc.).

---

## 🙌 Contributions

Feel free to submit PRs, open issues, or suggest improvements!

---

## ⭐ Acknowledgments

Thanks to the embedded systems and DIY electronics community for inspiration and references.

---

Enjoy building and exploring the **Chroma-Sifter**!

👨‍💻 Author

R. P. T. Sandeepa Dilhara (electronic, communication, and IT undergraduate student )
