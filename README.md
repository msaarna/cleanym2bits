# CLEANYM2BITS ADPCM Encoder/Decoder

CLEANYM2BITS is a project that implements a 2-bit YM ADPCM variant encoder and decoder. The encoder features an exhaustive recursive look-ahead optimization that significantly improves audio quality compared to standard ADPCM algorithms, especially for devices with extremely low bandwidth or storage capabilities.

This project is a 2-bit variant of the original 4-bit CLEANYM encoder.

The output is a raw stream of 2-bit ADPCM samples, without any specific header or container format, making it ideal for direct integration into embedded systems or applications requiring a minimal footprint.

## Features

*   **2-bit YM ADPCM Variant Encoding:** Provides a very high compression ratio, suitable for highly constrained environments.
*   **Exhaustive Look-Ahead Optimization:** The encoder performs a deep, recursive search of all possible future encoding paths within a short window. This allows it to make optimal encoding decisions that minimize perceived distortion and improve sound fidelity beyond a standard greedy encoder.
*   **Advanced Pre-Processing:** Includes a suite of DSP tools to condition audio for the limitations of 2-bit ADPCM, including:
    *   **Vocal Priority (Noise Shaping):** Penalizes low-frequency drift to keep vocals clear while pushing noise to high frequencies.
    *   **Dynamic Low-Pass Filter:** Automatically adjusts filtering based on signal intensity to prevent slope overload.
    *   **Slew Rate Limiter:** Enforces the codec's physical speed limits to prevent crackling.
    *   **Mid-Range Scoop:** "Hollows out" muddy frequencies to clarify the mix.
    *   **Compressor & Soft Clipper:** Tames dynamic range and rounds off peaks for a louder, cleaner signal.
*   **Raw Sample Stream Output:** Generates a direct stream of ADPCM samples, packed four to a byte, eliminating overhead for header parsing.
*   **High-Quality Decoding:** A robust decoder to convert the 2-bit ADPCM stream back into PCM audio.
*   **Resampling Support:** The encoder can resample input audio to a target rate or preserve the original sample rate.

## Why CLEANYM2BITS?

Traditional ADPCM encoders often make decisions based only on the current sample, leading to "greedy" choices that can propagate errors or introduce noticeable artifacts. CLEANYM2BITS's exhaustive look-ahead mechanism allows the encoder to anticipate future signal changes and choose the sequence of ADPCM codes that minimizes distortion over a multi-sample window, resulting in a cleaner reproduction of the original audio. This is particularly beneficial for applications where every bit of quality matters within the tightest possible constraints.

The look-ahead is entirely implemented in the encoder, with the decoder remaining simple. This allows for usage in applications where a specific YM ADPCM hardware variant is targeted, offloading the heavy computational work to the offline encoding step.

CLEANYM2BITS typically produces output samples in the quality neighborhood of 6-bit PCM. If you're looking for a higher quality output, you'll need more bits - check out the 4-bit CLEANYM project, which often exceeds 9-bit PCM quality.

## Build Instructions

To build CLEANYM2BITS, you will need a C compiler (like GCC or Clang) and the `libsndfile` and `libsamplerate` development libraries.

On Debian/Ubuntu:

```bash
sudo apt-get update
sudo apt-get install build-essential libsndfile1-dev libsamplerate0-dev
```

Then, compile the encoder and decoder using the provided `Makefile`:

```bash
make
```

## Usage

### Encoder (`cleanym2_encoder`)

Encodes a WAV file (or other `libsndfile`-supported formats) into a raw 2-bit YM ADPCM stream.

```bash
./cleanym2_encoder -i <input_audio_file> -o <output_adpcm_file> [-r <rate> | -R] [options]
```

**Core Options:**

*   `-i <input_audio_file>`: Specifies the input audio file (e.g., `input.wav`). **Required.**
*   `-o <output_adpcm_file>`: Specifies the output raw ADPCM file (e.g., `output.adpcm2`). **Required.**
*   `-r <rate>`: Specifies the output sample rate in samples/second. Default is `32160`.
*   `-R`: Uses the input file's sample rate as the encoding rate, overriding `-r`.

**Encoder Tuning:**

*   `-p <PRESET>`: Load a configuration preset (see below).
*   `-L <DEPTH>`: Lookahead Depth (1-6). Default is 3. Higher values find better paths but encode slower.

**Audio Pre-Processing:**

*   `-C <RATIO>`: Compressor. Limits dynamic range with RATIO (e.g., `4.0`).
*   `-M <DB>`: Mid-Range Scoop. Cuts 1kHz by DB amount (e.g., `6.0`) to remove "mud".
*   `-V <FACTOR>`: Vocal Priority. Penalizes low-frequency distortion (drift) to keep vocals clear (e.g., `5.0`).
*   `-s`: Enable Soft Clipping (Tanh Limiter) to round off peaks smoothly.
*   `-l <FREQ>`: Static Low-Pass Filter at FREQ (Hz).
*   `-D <MIN,MAX>`: Dynamic Low-Pass Filter. Lowers cutoff when loud to prevent distortion (e.g., `3000,12000`).
*   `-S <FACTOR>`: Slew Rate Limiter. Prevents the signal from moving faster than the codec can track (e.g., `0.20`).

**Presets:**

*   `music`: Enables Vocal Priority (`10.0`), Soft Clip, and a Static LPF at 70% Nyquist. Good for general tracks.
*   `speech`: Enables High Vocal Priority (`15.0`), Compression (`4:1`), Mid Scoop (`3dB`), and Soft Clip. Optimized for dialogue.
*   `deep`: Sets Lookahead Depth to 5 for maximum precision. Use with other flags.

**Example:**

```bash
./cleanym2_encoder -i my_song.wav -o my_song.adpcm2 -r 16000 -p music
```

### Decoder (`cleanym2_decoder`)

Decodes a raw 2-bit YM ADPCM stream back into a WAV file.

```bash
./cleanym2_decoder -i <input_adpcm_file> -o <output_wav_file> [-r <rate>]
```

**Options:**

*   `-i <input_adpcm_file>`: Specifies the input raw ADPCM file. **Required.**
*   `-o <output_wav_file>`: Specifies the output WAV file. **Required.**
*   `-r <rate>`: Specifies the sample rate of the output WAV file. This **must match** the rate used during encoding. Default is `14000`.

**Example:**

```bash
./cleanym2_decoder -i my_song.adpcm2 -o decoded_my_song.wav -r 16000
```

## ADPCM Stream Format

The output ADPCM file is a simple byte stream where each byte contains four 2-bit ADPCM samples. The samples are packed in the order they are processed, from the least significant bits to the most significant bits.

*   **Sample 1:** Bits 0-1
*   **Sample 2:** Bits 2-3
*   **Sample 3:** Bits 4-5
*   **Sample 4:** Bits 6-7

For example, if the encoder processes four samples with 2-bit values `0x1`, `0x3`, `0x0`, `0x2`, the resulting byte would be `10001101` in binary, or `0x8D`.

## Reference Implementations

For the C reference implementation of the decoder logic, please refer to the `decodesample_ym2` function in `cleanym2_decoder.c`.

Below are optimized assembly implementations for decoding a single byte (4 samples) on ARM processors.

### Shared Data Section
```asm
.data
.align 2
    @ The decoder state variables. 
    @ Initialize these to: step=127, history=0 before first use.
decoder_state:
    .short  127     @ step_size (offset 0)
    .short  0       @ history   (offset 2)
```

### ARMv7 Assembly (ARM State)
*Optimized for performance on Cortex-A / Cortex-R.*

```asm
.text
.arm
.align 4
.global decode_byte_arm

@ R0: Input Byte, R1: Output Buffer (int16_t*)
decode_byte_arm:
    PUSH    {R4-R6, LR}
    LDR     R4, =decoder_state
    LDRSH   R2, [R4, #0]        @ R2 = step
    LDRSH   R3, [R4, #2]        @ R3 = history
    MOV     R6, #4              @ Loop counter

loop_arm:
    AND     R5, R0, #3          @ Extract 2-bit sample
    LSR     R0, R0, #2          @ Shift input
    
    @ Calculate Diff
    TST     R5, #1
    MOV     R12, R2
    ADDNE   R12, R12, R12, LSL #1 @ If Delta=1, step*3
    LSR     R12, R12, #3        @ diff = (step*x) >> 3

    @ Update History
    TST     R5, #2
    SUBNE   R3, R3, R12
    ADDEQ   R3, R3, R12
    SSAT    R3, #16, R3         @ Clamp History

    STRH    R3, [R1], #2        @ Store Output

    @ Update Step
    TST     R5, #1
    MOV     R12, #235
    MOVNE   R12, #341
    MUL     R2, R2, R12
    LSR     R2, R2, #8          @ step = (step * mult) >> 8

    @ Clamp Step (127 - 24576)
    CMP     R2, #127
    MOVLT   R2, #127
    MOVW    R12, #24576
    CMP     R2, R12
    MOVGT   R2, R12

    SUBS    R6, R6, #1
    BNE     loop_arm

    STRH    R2, [R4, #0]
    STRH    R3, [R4, #2]
    POP     {R4-R6, PC}
```

### Thumb-2 Assembly
*Optimized for code density on Cortex-M.*

```asm
.text
.thumb
.syntax unified
.align 2
.global decode_byte_thumb

@ R0: Input Byte, R1: Output Buffer (int16_t*)
decode_byte_thumb:
    PUSH    {R4-R7, LR}
    LDR     R4, =decoder_state
    LDRSH   R2, [R4, #0]        @ R2 = step
    LDRSH   R3, [R4, #2]        @ R3 = history
    MOV     R7, #4              @ Loop counter

loop_thumb:
    AND     R5, R0, #3          @ Extract 2-bit sample
    LSR     R0, R0, #2          @ Shift input

    @ Calculate Diff
    MOV     R6, R2
    TST     R5, #1
    IT      NE
    ADDNE   R6, R6, R6, LSL #1  @ If Delta=1, step*3
    LSR     R6, R6, #3          @ diff

    @ Update History
    TST     R5, #2
    ITE     NE
    SUBNE   R3, R3, R6
    ADDEQ   R3, R3, R6
    SSAT    R3, #16, R3         @ Clamp History

    STRH    R3, [R1], #2        @ Store Output

    @ Update Step
    TST     R5, #1
    ITE     EQ
    MOVEQ   R6, #235
    MOVNE   R6, #341
    MUL     R2, R2, R6
    LSR     R2, R2, #8          @ step >> 8

    @ Clamp Step
    CMP     R2, #127
    IT      LT
    MOVLT   R2, #127
    MOVW    R6, #24576
    CMP     R2, R6
    IT      GT
    MOVGT   R2, R6

    SUBS    R7, R7, #1
    BNE     loop_thumb

    STRH    R2, [R4, #0]
    STRH    R3, [R4, #2]
    POP     {R4-R7, PC}
```
