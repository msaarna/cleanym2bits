# CLEANYM2BITS ADPCM Encoder/Decoder

CLEANYM2BITS is a project that implements a 2-bit YM ADPCM variant encoder and decoder. The encoder features a highly advanced, exhaustive recursive look-ahead optimization that significantly improves audio quality compared to standard ADPCM algorithms, especially for devices with extremely low bandwidth or storage capabilities.

This project is a 2-bit variant of the original 4-bit CLEANYM encoder.

The output is a raw stream of 2-bit ADPCM samples, without any specific header or container format, making it ideal for direct integration into embedded systems or applications requiring a minimal footprint.

## Features

*   **2-bit YM ADPCM Variant Encoding:** Provides a very high compression ratio, suitable for highly constrained environments.
*   **Exhaustive Look-Ahead Optimization:** The encoder performs a deep, recursive search of all possible future encoding paths within a short window. This allows it to make optimal encoding decisions that minimize perceived distortion and improve sound fidelity far beyond a standard greedy encoder.
*   **Raw Sample Stream Output:** Generates a direct stream of ADPCM samples, packed four to a byte, eliminating overhead for header parsing.
*   **High-Quality Decoding:** A robust decoder to convert the 2-bit ADPCM stream back into PCM audio.
*   **Resampling Support:** The encoder can resample input audio to a target rate or preserve the original sample rate.
*   **Psychoacoustic Error Metric:** The look-ahead system uses a weighted psychoacoustic model to prioritize errors that are more noticeable to the human ear, focusing on absolute error and volatility (changes in error).

## Why CLEANYM2BITS?

Traditional ADPCM encoders often make decisions based only on the current sample, leading to "greedy" choices that can propagate errors or introduce noticeable artifacts. CLEANYM2BITS's exhaustive look-ahead mechanism allows the encoder to anticipate future signal changes and choose the sequence of ADPCM codes that minimizes distortion over a multi-sample window, resulting in a cleaner reproduction of the original audio. This is particularly beneficial for applications where every bit of quality matters within the tightest possible constraints.

The look-ahead is entirely implemented in the encoder, with the decoder remaining simple. This allows for usage in applications where a specific YM ADPCM hardware variant is targeted, offloading the heavy computational work to the offline encoding step.

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

Or manually:
```bash
gcc cleanym2_encoder.c -o cleanym2_encoder -lsndfile -lsamplerate -lm -g
gcc cleanym2_decoder.c -o cleanym2_decoder -lsndfile -lm -g
```

## Usage

### Encoder (`cleanym2_encoder`)

Encodes a WAV file (or other `libsndfile`-supported formats) into a raw 2-bit YM ADPCM stream.

```bash
./cleanym2_encoder -i <input_audio_file> -o <output_adpcm_file> [-r <rate> | -R]
```

**Options:**

*   `-i <input_audio_file>`: Specifies the input audio file (e.g., `input.wav`). **Required.**
*   `-o <output_adpcm_file>`: Specifies the output raw ADPCM file (e.g., `output.adpcm2`). **Required.**
*   `-r <rate>`: Specifies the output sample rate in samples/second. Default is `32160`.
*   `-R`: Uses the input file's sample rate as the encoding rate, overriding `-r`.

**Example:**

```bash
./cleanym2_encoder -i my_song.wav -o my_song.adpcm2 -r 16000
```
This will encode `my_song.wav` to a 16kHz 2-bit ADPCM stream and save it as `my_song.adpcm2`.

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
This will decode `my_song.adpcm2` (assuming it was encoded at 16kHz) and save it as `decoded_my_song.wav`.

## ADPCM Stream Format

The output ADPCM file is a simple byte stream where each byte contains four 2-bit ADPCM samples. The samples are packed in the order they are processed, from the least significant bits to the most significant bits.

*   **Sample 1:** Bits 0-1
*   **Sample 2:** Bits 2-3
*   **Sample 3:** Bits 4-5
*   **Sample 4:** Bits 6-7

For example, if the encoder processes four samples with 2-bit values `0x1`, `0x3`, `0x0`, `0x2`, the resulting byte would be `10001101` in binary, or `0x8D`. The decoder processes them in the same order.

If the total number of samples is not a multiple of four, the last byte will contain the remaining one, two, or three valid 2-bit ADPCM samples in the lower bits, with the unused upper bits being zero.
