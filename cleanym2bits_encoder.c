#define PROGNAME "CLEANYM2 Encoder v1.1, by Mike Saarna. 2025."

#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <float.h>

#include <samplerate.h>

#define SYNC_QUALITY SRC_SINC_MEDIUM_QUALITY

// Default Lookahead. Now variable via -L.
int g_lookahead_depth = 3;
#define MAX_LOOKAHEAD_DEPTH 6

// Weights for the enhanced psychoacoustic error metric.
#define ERROR_WEIGHT_ABSOLUTE   1.00f
#define ERROR_WEIGHT_VOLATILITY 2.00f 

long TARGETRATE = 32160;

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#include <unistd.h>

#define FRAMESIZE 64
long SAMPLERATE;

enum outputformats
{ RAWFMT = 0, ASMFMT = 1, CFMT = 2 };
int outputformat = RAWFMT;

int64_t samplecount;
float *samplebuffer = NULL;
FILE *outfile;

// --- Biquad Filter Structures ---
typedef struct {
    float a0, a1, a2, b0, b1, b2;
    float z1, z2;
} Biquad;

// Global Vocal Priority Factor (set via -V)
float vocalPriorityFactor = 0.0f;

int loadwave (char *filename, int preserveRate);
void usage (char *programname);
float getbitrate (long noisescore, int64_t samplecount);
float normalizeSample (float *normBuffer, int64_t normBufferSize);
void normalize_buffer_to_peak (float *buffer, int64_t num_samples, float target_peak);

void apply_hipass_filter_to_buffer (float *buffer, int num_samples, float cutoff_freq_hz, float sample_rate_hz);
void apply_lowpass_filter_to_buffer (float *buffer, int num_samples, float cutoff_freq_hz, float sample_rate_hz);
void apply_soft_clip_to_buffer (float *buffer, int64_t num_samples);
void apply_dynamic_lowpass_filter (float *buffer, int64_t num_samples, float min_freq, float max_freq, float sample_rate);
void apply_slew_limiter (float *buffer, int64_t num_samples, float slew_factor);
void apply_mid_scoop (float *buffer, int64_t num_samples, float db_cut, float sample_rate);
void apply_compressor (float *buffer, int64_t num_samples, float ratio, float threshold_db, float sample_rate);

float returnMedianValue (float a, float b, float c, float d, float e);
int compare_floats (const void *a, const void *b);

uint8_t encodesample_ym2 (int16_t RawSample);
int16_t decodesample_ym2 (uint8_t AdpcmSample);
uint8_t encodesample_ym2_lookahead (int64_t current_sample_index, int16_t CurrentRawSample, long lookbehind_error, float lookbehind_smoothed_error);

// New helper for state-agnostic simulation
int16_t decodesample_ym2_simulate(uint8_t AdpcmSample, int16_t initial_step_size, int32_t initial_history, int16_t *out_next_step_size, int32_t *out_next_history);
// New recursive helper for exhaustive search
float find_best_future_score(int64_t sample_index, int depth, long previous_error, float previous_smoothed_error, int16_t current_step_size, int32_t current_history);

void save_all (void);
void save_encoder_state (void);
void save_decoder_state (void);
void restore_all (void);
void restore_encoder_state (void);
void restore_decoder_state (void);

// used for stats
long noisescore = 0;
long framenoise = 0;
long worstframenoise = 0;
long fixcount = 0;

// used for normalisation
int8_t volumeDivisor = 1;

// Max slew rate default. Smaller values = more aggressive limiting.
// Used if -S is passed without argument, or as base.
float max_slew_rate_factor = 0.20f; 

// step table used for 2-bit Encoder and Decoder
const int step_table_2bit[2] = { 235, 341 };

// we keep these as globals, to easily save and restore for A:B testing
int16_t encoder_step_size = 127;
int32_t encoder_history = 0;
int16_t decoder_step_size = 127;
int32_t decoder_history = 0;

int16_t save_encoder_step_size;
int32_t save_encoder_history;
int16_t save_decoder_step_size;
int32_t save_decoder_history;

// Define M_PI if not available (e.g., on MSVC before C++20 or in strict C modes)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Hi-pass filter state 
static float hp_x_prev = 0.0f;	// Previous input sample x[n-1]
static float hp_y_prev = 0.0f;	// Previous output sample y[n-1]
static float hp_alpha = 0.0f;	// Filter coefficient, calculated once

// Low-pass filter state
static float lp_y_prev = 0.0f;
static float lp_alpha = 0.0f;

int main (int argc, char **argv)
{
    int c;
    int64_t t;
    extern char *optarg;
    extern int optind, optopt;
    int errflg = 0;

    int preserveRate = 0;
    int enableSoftClip = 0;
    float lpfCutoff = 0.0f;
    
    // Dynamic LPF params
    float dynLpfMin = 0.0f;
    float dynLpfMax = 0.0f;
    int dynLpfEnabled = 0;

    // Slew Limiter params
    float slewFactor = 0.0f;

    // Mid Scoop params
    float midScoopDb = 0.0f;
    
    // Compressor
    float compRatio = 0.0f;

    // Preset flags
    int presetMusic = 0;

    char *infilename, *outfilename;

    infilename = NULL;
    outfilename = NULL;

    // Added L: (Lookahead), C: (Compressor), p: (Preset)
    while ((c = getopt (argc, argv, ":i:o:f:r:Rsl:D:S:M:V:L:C:p:")) != -1)
    {
	switch (c)
	{
	case 'i':
	    infilename = optarg;
	    break;
	case 'o':
	    outfilename = optarg;
	    break;
	case 'r':
	    TARGETRATE = atoi (optarg);
	    break;
	case 'R':
	    preserveRate = 1;
	    break;
    case 's':
        enableSoftClip = 1;
        break;
    case 'l':
        lpfCutoff = atof(optarg);
        break;
    case 'D':
        {
            char *comma = strchr(optarg, ',');
            if (comma) {
                *comma = '\0';
                dynLpfMin = atof(optarg);
                dynLpfMax = atof(comma + 1);
                dynLpfEnabled = 1;
            } else {
                fprintf(stderr, "Error: -D requires format MIN_FREQ,MAX_FREQ (e.g. -D 3000,12000)\n");
                errflg++;
            }
        }
        break;
    case 'S':
        slewFactor = atof(optarg);
        break;
    case 'M':
        midScoopDb = atof(optarg); // e.g., 6.0 for 6dB cut
        break;
    case 'V':
        vocalPriorityFactor = atof(optarg); // e.g., 5.0 for strong vocal priority
        break;
    case 'L':
        g_lookahead_depth = atoi(optarg);
        if (g_lookahead_depth < 1) g_lookahead_depth = 1;
        if (g_lookahead_depth > MAX_LOOKAHEAD_DEPTH) g_lookahead_depth = MAX_LOOKAHEAD_DEPTH;
        break;
    case 'C':
        compRatio = atof(optarg); // e.g., 4.0
        break;
    case 'p':
        if (strcmp(optarg, "music") == 0) {
            // Music: Vocal Priority, Soft Clip, LPF (70% Nyquist)
            vocalPriorityFactor = 10.0f;
            enableSoftClip = 1;
            presetMusic = 1; // LPF calculated later
        } else if (strcmp(optarg, "deep") == 0) {
            // Deep: High lookahead
            g_lookahead_depth = 5;
        } else if (strcmp(optarg, "speech") == 0) {
            // Speech: High Vocal Priority, Compressor, Mid Scoop
            vocalPriorityFactor = 15.0f;
            compRatio = 4.0f;
            midScoopDb = 3.0f;
            enableSoftClip = 1;
        } else {
            fprintf(stderr, "Unknown preset: %s\n", optarg);
            errflg++;
        }
        break;
	case ':':
	    fprintf (stderr, "*** ERR: option -%c requires an operand\n", optopt);
	    errflg++;
	    break;
	case '?':
	    fprintf (stderr, "*** ERR: unrecognised option \"-%c\"\n", optopt);
	    errflg++;
	    break;
	}
    }
    if (errflg)
    {
	usage (argv[0]);
	return (2);
    }

    if (infilename == NULL)
    {
	fprintf (stderr, "-i wav file parameter required\n\n");
	usage (argv[0]);
	return 1;
    }

    if (outfilename == NULL)
    {
	fprintf (stderr, "-o output file parameter required\n\n");
	usage (argv[0]);
	return 1;

    }

    printf ("Loading file %s\n", infilename);
    if (loadwave (infilename, preserveRate) != 0)
    {
	fprintf (stderr, "error: loading wav file failed\n");
	// Ensure samplebuffer is freed if loadwave fails after allocation
	if (samplebuffer != NULL)
	{
	    free (samplebuffer);
	    samplebuffer = NULL;
	}
	return 1;
    }

    // Apply preset logic that depends on final sample rate
    if (presetMusic && lpfCutoff == 0.0f) {
        lpfCutoff = (float)TARGETRATE * 0.5f * 0.8f;
        printf("Preset 'music' set LPF to %.0f Hz\n", lpfCutoff);
    }

    // --- Pre-processing Stage ---
    // Order matters:
    // 1. Mid Scoop (Hollow out the complex middle)
    // 2. Compressor (Control dynamics early)
    // 3. LPF (Shape the tone)
    // 4. Slew Limiter (Enforce limits)
    // 5. Soft Clip (Catch peaks)

    if (midScoopDb > 0.0f)
    {
        printf("Applying Mid-Range Scoop (-%.1fdB at 1kHz)...\n", midScoopDb);
        apply_mid_scoop(samplebuffer, samplecount, midScoopDb, (float)TARGETRATE);
    }

    if (compRatio > 1.0f)
    {
        printf("Applying Compressor (Ratio %.1f:1)...\n", compRatio);
        // Threshold fixed at -12dB for general utility
        apply_compressor(samplebuffer, samplecount, compRatio, -12.0f, (float)TARGETRATE);
    }

    if (dynLpfEnabled)
    {
        printf("Applying Dynamic Low-Pass Filter (%.0fHz - %.0fHz)...\n", dynLpfMin, dynLpfMax);
        apply_dynamic_lowpass_filter(samplebuffer, samplecount, dynLpfMin, dynLpfMax, (float)TARGETRATE);
    }
    else if (lpfCutoff > 0.0f)
    {
        printf("Applying Static Low-Pass Filter at %.0f Hz...\n", lpfCutoff);
        apply_lowpass_filter_to_buffer(samplebuffer, samplecount, lpfCutoff, (float)TARGETRATE);
    }

    if (slewFactor > 0.0f)
    {
        printf("Applying Slew Rate Limiter (Factor: %.2f)...\n", slewFactor);
        apply_slew_limiter(samplebuffer, samplecount, slewFactor);
    }

    if (enableSoftClip)
    {
        printf("Applying Soft Clip (Tanh Limiter)...\n");
        apply_soft_clip_to_buffer(samplebuffer, samplecount);
    }

    // --- End Pre-processing ---

    outfile = fopen (outfilename, "wb");
    if (outfile == NULL)
    {
	fprintf (stderr, "error: creation of output file \"%s\" failed\n", outfilename);
	if (samplebuffer != NULL)
	    free (samplebuffer);
	return 1;
    }

    uint8_t AdpcmSample = 0;
    uint8_t AdpcmSamplePacked = 0;
    int16_t SourcePcmSample, PcmSample;
    long last_actual_error = 0; // For look-behind error tracking.
    float last_smoothed_error = 0.0f; // For Vocal Priority noise shaping

    float *frame_distortions = calloc ((samplecount / FRAMESIZE) + 1, sizeof (float));
    if (frame_distortions == NULL)
    {
	fprintf (stderr, "Could not allocate memory for tracking distortions.\n");
	fclose (outfile);
	if (samplebuffer != NULL)
	    free (samplebuffer);
	return 1;
    }
    
    printf("Encoding with Lookahead Depth: %d\n", g_lookahead_depth);

    for (t = 0; t < samplecount; t++)
    {

	if ((t % FRAMESIZE) == 0 && t > 0)
	{
	    // end of a frame - record info for noise stats...
	    if (framenoise > worstframenoise)
		worstframenoise = framenoise;

	    // Ensure index is within bounds for frame_distortions
	    if ((t / FRAMESIZE) < (samplecount / FRAMESIZE) + 1)
	    {			// Check should be against allocated size
		frame_distortions[t / FRAMESIZE] = (float) framenoise / FRAMESIZE;
	    }

	    framenoise = 0;
	}

	SourcePcmSample = (int16_t) samplebuffer[t];	// Fetch this wav sample (already scaled to int16 range as float).

	// Use the lookahead encoder if there's at least one next sample.
	// The lookahead function itself handles cases where fewer than (LOOKAHEAD_DEPTH-1) future samples are available.
	if (t < samplecount - 1)
	{	// If there's at least one next sample for lookahead
        // Pass the last actual error to the lookahead function.
	    AdpcmSample = encodesample_ym2_lookahead (t, SourcePcmSample, last_actual_error, last_smoothed_error);
	}
	else
	{	// For the very last sample, no lookahead is possible with this scheme; use normal encoder.
	    AdpcmSample = encodesample_ym2 (SourcePcmSample);
	}


	// Run the decoder, just for the upcoming noise stats.
	// The encodesample_ym2_lookahead function ensures that global decoder_step_size/history
	// are restored to their state *before* this current sample's AdpcmSample was determined.
	// So, this decodesample_ym2 call uses the correct initial decoder state and advances it,
	// keeping it synchronized with the encoder's state progression.
	PcmSample = (int16_t) decodesample_ym2 (AdpcmSample);

    // Calculate and store the actual error for the next iteration's look-behind.
    long current_error = (long)PcmSample - (long)SourcePcmSample;
    last_actual_error = current_error;

    // Update the smoothed error (Low-Pass Filter on the error signal)
    // Alpha 0.15 gives a good balance for vocal range tracking
    last_smoothed_error = last_smoothed_error + 0.15f * ((float)current_error - last_smoothed_error);

	noisescore += abs (current_error);
	framenoise += abs (current_error);

    // Pack four 2-bit samples into a byte.
    // The bits are packed in the order they are processed.
    // Sample 0: bits 1,0
    // Sample 1: bits 3,2
    // Sample 2: bits 5,4
    // Sample 3: bits 7,6
    AdpcmSamplePacked |= (AdpcmSample & 0x03) << ((t % 4) * 2);

    // Write the byte every 4 samples, or if it's the last sample.
    if ((t % 4) == 3)
    {
        fputc(AdpcmSamplePacked, outfile);
        AdpcmSamplePacked = 0; // Reset for the next byte.
    }

    } // End of for loop over samples

    // If the total number of samples is not a multiple of 4,
    // write the last partially filled byte.
    if ((samplecount % 4) != 0)
    {
        fputc(AdpcmSamplePacked, outfile);
    }

    fclose (outfile);

    printf ("Lookahead optimisation adjusted %ld 2-bit samples. (%.2f%%)\n", fixcount, ((float)fixcount/(float)samplecount)*100.0f);
    printf ("Wrote %lld 2-bit samples, packed into %lld bytes.\n", (long long) samplecount, (long long) (samplecount + 3) / 4);

    if (samplecount / FRAMESIZE > 0)
    {
	qsort (frame_distortions, samplecount / FRAMESIZE, sizeof (float), compare_floats);
    }

    float avgbits, worstbits, medianbits;

    avgbits = getbitrate (noisescore, samplecount);
    worstbits = getbitrate (worstframenoise, (FRAMESIZE > 0 ? FRAMESIZE : 1));

    printf ("\n");
    printf ("ADPCM quality report...\n");

    if (samplecount / FRAMESIZE > 0)
    {
	medianbits = getbitrate ((long) frame_distortions[(samplecount / FRAMESIZE) / 2], 1);	// Median distortion is per-sample average for that frame
	printf ("  avg    distortion: %*ld    avg bits/sample:    %2.2f\n", 6,
		(samplecount > 0 ? noisescore / samplecount : 0L), avgbits);
	printf ("  median distortion: %*ld    median bits/sample: %2.2f\n", 6,
		(long) frame_distortions[(samplecount / FRAMESIZE) / 2], medianbits);
	printf ("  worst  distortion: %*ld    worst bits/sample:  %2.2f\n", 6,
		(FRAMESIZE > 0 ? worstframenoise / FRAMESIZE : 0L), worstbits);
    }
    else
    {
	medianbits = 0.0f;
	printf ("  avg    distortion: %*ld    avg bits/sample:    %2.2f\n", 6,
		(samplecount > 0 ? noisescore / samplecount : 0L), avgbits);
	printf ("  (median distortion not available for very short samples)\n");
	printf ("  worst  distortion: %*ld    worst bits/sample:  %2.2f\n", 6,
		(FRAMESIZE > 0 && worstframenoise > 0 ? worstframenoise / FRAMESIZE : 0L), worstbits);
    }
    printf ("\n");
    printf ("  Note: ADPCM bits/sample aren't directly comparable to PCM bits/sample.\n");
    printf ("  ADPCM noise increases as volume changes, masking it's perception.\n");
    

    free (frame_distortions);
    if (samplebuffer != NULL)
    {
	free (samplebuffer);
	samplebuffer = NULL;
    }
    return (0);
}

// A pure function to simulate one step of the YM2 decoder without using global state.
// It takes the initial state and returns the resulting state via output pointers.
int16_t decodesample_ym2_simulate(uint8_t AdpcmSample, int16_t initial_step_size, int32_t initial_history, int16_t *out_next_step_size, int32_t *out_next_history)
{
    int32_t sign, delta, nstep, diff;
    int16_t current_step_size = initial_step_size;
    int32_t current_history = initial_history;

    sign = AdpcmSample & 2;
    delta = AdpcmSample & 1;
    diff  = ((1+(delta<<1)) * current_step_size) >> 3;
    nstep = (step_table_2bit[delta] * current_step_size) >> 8;
    diff = CLAMP(diff, 0, 32767);
    if (sign > 0)
        current_history -= diff;
    else
        current_history += diff;
    
    current_step_size = CLAMP(nstep, 127, 24576);
    current_history = CLAMP(current_history, -32768, 32767);

    *out_next_step_size = current_step_size;
    *out_next_history = current_history;

    return (int16_t)current_history;
}

// Recursively finds the minimum possible psychoacoustic score for a path of g_lookahead_depth.
float find_best_future_score(int64_t sample_index, int depth, long previous_error, float previous_smoothed_error, int16_t current_step_size, int32_t current_history)
{
    // Base case: If we've looked far enough ahead or are at the end of the sample, stop.
    if (depth >= g_lookahead_depth || sample_index >= samplecount) {
        return 0.0f;
    }

    float min_score_for_this_level = FLT_MAX;
    int16_t raw_sample = (int16_t)samplebuffer[sample_index];
    
    uint8_t trial_adpcm;
    for (trial_adpcm = 0; trial_adpcm <= 3; ++trial_adpcm) {
        int16_t next_step_size;
        int32_t next_history;
        int16_t decoded_pcm = decodesample_ym2_simulate(trial_adpcm, current_step_size, current_history, &next_step_size, &next_history);
        
        long current_error = (long)decoded_pcm - (long)raw_sample;
        
        // Calculate smoothed error for this path (Noise Shaping)
        // Alpha 0.15 tracks the vocal range
        float current_smoothed_error = previous_smoothed_error + 0.15f * ((float)current_error - previous_smoothed_error);

        float absolute_error = fabsf((float)current_error);
        float volatility_error = fabsf((float)current_error - (float)previous_error);
        
        // Base Score
        float current_step_score = (ERROR_WEIGHT_ABSOLUTE * absolute_error) + (ERROR_WEIGHT_VOLATILITY * volatility_error);
        
        // Vocal Priority Penalty (Noise Shaping)
        // Penalize error that accumulates in the low frequencies
        if (vocalPriorityFactor > 0.0f) {
            current_step_score += vocalPriorityFactor * fabsf(current_smoothed_error);
        }

        // Recurse to find the best possible score for the rest of the path
        float future_score = find_best_future_score(sample_index + 1, depth + 1, current_error, current_smoothed_error, next_step_size, next_history);
        
        float total_path_score = current_step_score + future_score;

        if (total_path_score < min_score_for_this_level) {
            min_score_for_this_level = total_path_score;
        }
    }

    return min_score_for_this_level;
}


// Top-level lookahead function that uses an exhaustive recursive search to find the best ADPCM value.
uint8_t encodesample_ym2_lookahead (int64_t current_sample_index, int16_t CurrentRawSample, long lookbehind_error, float lookbehind_smoothed_error)
{
    uint8_t best_adpcm_for_current_sample = 0;
    float min_total_score = FLT_MAX;

    // Snapshot of global states at function entry.
    int16_t entry_encoder_step_size = encoder_step_size;
    int32_t entry_encoder_history = encoder_history;
    int16_t entry_decoder_step_size = decoder_step_size;
    int32_t entry_decoder_history = decoder_history;

    // For fixcount statistics: determine what the normal encoder would have chosen.
    save_all();
    uint8_t normal_adpcm_for_current_sample = encodesample_ym2(CurrentRawSample);
    restore_all();

    uint8_t trial_adpcm_current;
    for (trial_adpcm_current = 0; trial_adpcm_current <= 3; ++trial_adpcm_current)
    {
        // --- Part 1: Simulate the current sample with this trial ADPCM value ---
        int16_t next_step_size;
        int32_t next_history;
        int16_t decoded_pcm_current = decodesample_ym2_simulate(trial_adpcm_current, entry_encoder_step_size, entry_encoder_history, &next_step_size, &next_history);

        long current_error = (long)decoded_pcm_current - (long)CurrentRawSample;

        // Noise shaping update for this trial
        float current_smoothed_error = lookbehind_smoothed_error + 0.15f * ((float)current_error - lookbehind_smoothed_error);

        // --- Part 2: Calculate the score for this first step ---
        float absolute_error = fabsf((float)current_error);
        float volatility_error = fabsf((float)current_error - (float)lookbehind_error);
        float current_score = (ERROR_WEIGHT_ABSOLUTE * absolute_error) + (ERROR_WEIGHT_VOLATILITY * volatility_error);
        
        // Vocal Priority Penalty
        if (vocalPriorityFactor > 0.0f) {
            current_score += vocalPriorityFactor * fabsf(current_smoothed_error);
        }

        // --- Part 3: Recursively find the best possible score for the future path ---
        float future_score = find_best_future_score(current_sample_index + 1, 1, current_error, current_smoothed_error, next_step_size, next_history);

        float total_score = current_score + future_score;

        // --- Part 4: Compare total score and update best choice ---
        if (total_score < min_total_score)
        {
            min_total_score = total_score;
            best_adpcm_for_current_sample = trial_adpcm_current;
        }
    }

    // --- Finalize: Apply the best choice and set global states correctly ---
    if (normal_adpcm_for_current_sample != best_adpcm_for_current_sample)
        fixcount++;

    // Correct way to advance the *encoder's* state based on our decision:
    // The encoder state is just the decoder state. We simulate the decode of our chosen value.
    decoder_step_size = entry_encoder_step_size;
    decoder_history = entry_encoder_history;
    decodesample_ym2(best_adpcm_for_current_sample); 
    encoder_step_size = decoder_step_size;
    encoder_history = decoder_history;


    // Restore the *global decoder state* to what it was at function entry for the main loop's stats.
    decoder_step_size = entry_decoder_step_size;
    decoder_history = entry_decoder_history;

    return best_adpcm_for_current_sample;
}


float getbitrate (long noisescore, int64_t samplecount)
{
    // Ensure samplecount is not zero to prevent division by zero
    if (samplecount == 0)
    {
	return 0.0f;
    }

    long avgdistortion_long = noisescore / samplecount;

    // Handle case where average distortion is 0 (perfect encoding for this block)
    if (avgdistortion_long <= 0)
    {				// Can be 0 if noisescore is 0 or noisescore < samplecount
	return 16.0f;		// No bits lost to noise
    }

    float avgdistortion_float = (float) avgdistortion_long;

    // Calculate the number of bits needed to represent this average distortion.
    // This is log base 2 of the average distortion.
    // log2(x) = log(x) / log(2) using natural logarithm (log) or base-10 (log10)
    float bits_lost_to_noise = logf (avgdistortion_float) / logf (2.0f);

    // If average error is less than 1 LSB (in 16-bit context)
    // logf of value < 1 is negative. This means bits_lost_to_noise would be negative.
    // Consider such small errors as effectively zero bits lost.
    if (avgdistortion_float < 1.0f)
    {
	bits_lost_to_noise = 0.0f;
    }

    float effective_bitrate = 16.0f - bits_lost_to_noise;

    // Clamp effective_bitrate
    if (effective_bitrate < 0.0f)
    {
	effective_bitrate = 0.0f;
    }
    if (effective_bitrate > 16.0f) // highly unlikely
    {
	effective_bitrate = 16.0f;
    }

    return effective_bitrate;
}

float normalizeSample (float *normBuffer, int64_t normBufferSize)
{
    int64_t t;
    float maxvolume = 0.0f;
    float fVolumeDivisor;

    if (normBufferSize == 0)
	return 1.0f;		// Handle empty buffer case

    for (t = 0; t < normBufferSize; t++)
	if (fabsf (normBuffer[t]) > maxvolume)
	    maxvolume = fabsf (normBuffer[t]);

    if (maxvolume == 0.0f)	// Completely silent sample
    {
	fVolumeDivisor = 1.0f;	// No change needed
    }
    else if (maxvolume > 1.0f)	// Needs attenuation
    {
	// Attenuate to make peak 1.0f
	float attenuation_factor = 1.0f / maxvolume;
	for (t = 0; t < normBufferSize; t++)
	    normBuffer[t] = normBuffer[t] * attenuation_factor;
	fVolumeDivisor = 1.0f;
    }
    else			// Needs amplification (maxvolume is > 0 and <= 1.0)
    {
	// Amplify by a power of 2 to make peak close to 1.0f without exceeding it.
	fVolumeDivisor = exp2f (floorf (log2f (1.0f / maxvolume)));

	// Safety clamp for fVolumeDivisor, considering it's cast to int8_t later.
	// A very small maxvolume could lead to a huge fVolumeDivisor.
	if (fVolumeDivisor > 127.0f)
	    fVolumeDivisor = 127.0f;
	if (fVolumeDivisor < 1.0f)
	    fVolumeDivisor = 1.0f;	// Should not happen if maxvolume <= 1.0f and > 0

	for (t = 0; t < normBufferSize; t++)
	    normBuffer[t] = normBuffer[t] * fVolumeDivisor;
    }
    return fVolumeDivisor;
}

int compare_floats (const void *a, const void *b)
{
    float fa = *(const float *) a;
    float fb = *(const float *) b;
    if (fa < fb)
	return -1;
    if (fa > fb)
	return 1;
    return 0;
}


uint8_t encodesample_ym2 (int16_t RawSample)
{
    // The 2-bit YM ADPCM Encoder, adapted from the 4-bit YMZ ADPCM Encoder
    // https://github.com/superctr/adpcm/blob/master/ymz_codec.c

    int32_t sign, step, delta, nstep, diff;
    unsigned int AdpcmSampleVal;

    step = RawSample - encoder_history;

    if (encoder_step_size == 0)
    {
	    AdpcmSampleVal = (abs (step) == 0) ? 0 : 1;
    }
    else
    {
	    AdpcmSampleVal = (abs (step) << 16) / (encoder_step_size << 14);
    }
    AdpcmSampleVal = CLAMP (AdpcmSampleVal, 0, 1);

    if (step < 0)
	    AdpcmSampleVal = AdpcmSampleVal | 2;

    // adjust step size and history
    sign = AdpcmSampleVal & 2;
    delta = AdpcmSampleVal & 1;

    diff = ((1 + (delta << 1)) * encoder_step_size) >> 3;
    nstep = (step_table_2bit[delta] * encoder_step_size) >> 8;
    diff = CLAMP (diff, 0, 32767);
    if (sign > 0)
	    encoder_history -= diff;
    else
	    encoder_history += diff;

    encoder_step_size = CLAMP (nstep, 127, 24576);
    encoder_history = CLAMP (encoder_history, -32768, 32767);

    return (AdpcmSampleVal & 0x03);
}

int16_t decodesample_ym2 (uint8_t AdpcmSample)
{
    // The 2-bit YM ADPCM Decoder, adapted from the 4-bit YMZ ADPCM Decoder
    // https://github.com/superctr/adpcm/blob/master/ymz_codec.c

    int32_t sign, delta, nstep, diff;

    sign = AdpcmSample & 2;
    delta = AdpcmSample & 1;

    diff = ((1 + (delta << 1)) * decoder_step_size) >> 3;
    nstep = (step_table_2bit[delta] * decoder_step_size) >> 8;
    diff = CLAMP (diff, 0, 32767);
    if (sign > 0)
	    decoder_history -= diff;
    else
	    decoder_history += diff;

    decoder_step_size = CLAMP (nstep, 127, 24576);
    decoder_history = CLAMP (decoder_history, -32768, 32767);

    return ((int16_t) decoder_history);
}

int loadwave (char *filename, int preserveRate)
{
    SF_INFO sndInfo;
    SNDFILE *sndFile;
    long numFrames;
    int channels;
    float *samplebuffertmp;
    int64_t t_load;

    // Open sound file
    sndFile = sf_open (filename, SFM_READ, &sndInfo);
    if (sndFile == NULL)
    {
	fprintf (stderr, "Error reading wav file '%s': %s\n", filename, sf_strerror (sndFile));
	return 1;
    }

    channels = sndInfo.channels;

    SAMPLERATE = sndInfo.samplerate;
    if (preserveRate)
	TARGETRATE = SAMPLERATE;

    samplecount = sndInfo.frames;

    samplebuffer = calloc (samplecount, sizeof (float));	// Allocate for actual number of frames for single channel float
    if (samplebuffer == NULL)
    {
	fprintf (stderr, "Could not allocate memory for file (samplebuffer)\n");
	sf_close (sndFile);
	return 1;
    }

    // Load the sample data into a array of floats...
    if (channels == 1)
	numFrames = sf_readf_float (sndFile, samplebuffer, samplecount);	// Use samplecount (int64_t)
    else			// channels>1
    {
	// we have at least two channels, but we only want one.
	// drop all channels except the first (left) one...
	long s = 0;
	samplebuffertmp = calloc (samplecount * channels, sizeof (float));	// Allocate for all channels' data
	if (samplebuffertmp == NULL)
	{
	    if (samplebuffer != NULL)
		free (samplebuffer);
	    samplebuffer = NULL;	// Mark as freed
	    fprintf (stderr, "Could not allocate memory for file (samplebuffertmp)\n");
	    sf_close (sndFile);
	    return 1;
	}

	numFrames = sf_readf_float (sndFile, samplebuffertmp, samplecount);

	for (t_load = 0; t_load < numFrames; t_load++)
	{
	    samplebuffer[t_load] = samplebuffertmp[s];
	    s = s + channels;
	}
	free (samplebuffertmp);
    }

    // Check correct number of samples loaded
    if (numFrames != samplecount)	// Compare with samplecount (int64_t)
    {
	fprintf (stderr, "Did not read enough frames for source. Expected %lld, got %ld\n", (long long) samplecount,
		 numFrames);
	sf_close (sndFile);
	if (samplebuffer != NULL)
	    free (samplebuffer);
	samplebuffer = NULL;
	return 1;
    }
    sf_close (sndFile);

    if (TARGETRATE == SAMPLERATE)
	printf ("The input samplerate is equal to output sample rate.\nNo scaling required\n");
    else
    {
	double ratio = (double) TARGETRATE / SAMPLERATE;
	int64_t input_frames = samplecount;
	// Calculate output_frames carefully to avoid overflow if samplecount is large.
	int64_t output_frames = (int64_t) round ((double) samplecount * ratio);

	float *samplebufferout = calloc (output_frames, sizeof (float));
	if (samplebufferout == NULL)
	{
	    fprintf (stderr, "Could not allocate memory for resampled buffer\n");
	    if (samplebuffer != NULL)
		free (samplebuffer);
	    samplebuffer = NULL;
	    return 2;		// Or appropriate error code
	}


	printf ("Scaling sample rate from %ld to %ld...", SAMPLERATE, TARGETRATE);
	fflush (stdout);

	SRC_DATA src_data = {
	    .data_in = samplebuffer,
	    .input_frames = input_frames,
	    .data_out = samplebufferout,
	    .output_frames = output_frames,
	    .src_ratio = ratio
	};

	int error = src_simple (&src_data, SYNC_QUALITY, 1);	// 1 channel
	if (error)
	{
	    fprintf (stderr, "\nResampling error: %s\n", src_strerror (error));
	    free (samplebufferout);
	    if (samplebuffer != NULL)
		free (samplebuffer);
	    samplebuffer = NULL;
	    return 2;
	}

	free (samplebuffer);

	samplebuffer = samplebufferout;
	samplecount = src_data.output_frames_gen;


	printf (" done. Output frames: %lld\n", (long long) samplecount);
    }

    // In case we encountered overly-hot samples...
    float norm_factor = normalizeSample (samplebuffer, samplecount);
    volumeDivisor = (int8_t) roundf (norm_factor);	// roundf and cast to int8_t
    if (volumeDivisor == 0 && norm_factor > 0)
	volumeDivisor = 1;	// Avoid divisor being 0

    fprintf (stderr, "Applied temporary normalization (factor: %f, stored divisor: %d).\n", norm_factor, volumeDivisor);


    // In case we introduced some issues with processing, re-normalize to [-1,1]
    // and get amplification factor if any.
    normalizeSample (samplebuffer, samplecount);	// This normalizes to [-1,1] if needed.

    // scale down by the factor determined by the *first* normalization, if it was an amplification.
    if (volumeDivisor > 1)
    {				// Only apply if original normalization decided to amplify
	for (t_load = 0; t_load < samplecount; t_load++)
	    samplebuffer[t_load] = samplebuffer[t_load] / volumeDivisor;
    }

    // Scale up to 16-bit signed integer range for ADPCM encoding
    for (t_load = 0; t_load < samplecount; t_load++)
	samplebuffer[t_load] = CLAMP (samplebuffer[t_load] * 32767.0f, -32768.0f, 32767.0f);

    return 0;
}

void hipass_filter_setup (float cutoff_freq_hz, float sample_rate_hz)
{
    if (cutoff_freq_hz <= 0.0f || sample_rate_hz <= 0.0f || cutoff_freq_hz >= sample_rate_hz / 2.0f)
    {
	hp_alpha = 1.0f;	// Pass-through or disable if params are invalid/extreme
	hp_x_prev = 0.0f;
	hp_y_prev = 0.0f;
	return;
    }
    double rc = 1.0 / (2.0 * M_PI * cutoff_freq_hz);
    double dt = 1.0 / sample_rate_hz;
    hp_alpha = (float) (rc / (rc + dt));	// For y[i] = alpha * (y[i-1] + x[i] - x[i-1])

    hp_x_prev = 0.0f;
    hp_y_prev = 0.0f;
}

float hipass_filter_process (float input_sample)
{
    if (hp_alpha == 1.0f)
	return input_sample;	// Filter disabled or pass-through

    float output_sample = hp_alpha * (hp_y_prev + input_sample - hp_x_prev);
    hp_y_prev = output_sample;
    hp_x_prev = input_sample;
    return output_sample;
}

void apply_hipass_filter_to_buffer (float *buffer, int num_samples, float cutoff_freq_hz, float sample_rate_hz)
{
    hipass_filter_setup (cutoff_freq_hz, sample_rate_hz);
    int i;
    for (i = 0; i < num_samples; i++)
    {
	buffer[i] = hipass_filter_process (buffer[i]);
    }
}

void lowpass_filter_setup (float cutoff_freq_hz, float sample_rate_hz)
{
    if (cutoff_freq_hz <= 0.0f || sample_rate_hz <= 0.0f || cutoff_freq_hz >= sample_rate_hz / 2.0f)
    {
        lp_alpha = 1.0f; // Invalid params, pass through
        lp_y_prev = 0.0f;
        return;
    }
    
    double rc = 1.0 / (2.0 * M_PI * cutoff_freq_hz);
    double dt = 1.0 / sample_rate_hz;
    lp_alpha = (float)(dt / (rc + dt));
    lp_y_prev = 0.0f;
}

float lowpass_filter_process (float input_sample)
{
    float output_sample = lp_y_prev + lp_alpha * (input_sample - lp_y_prev);
    lp_y_prev = output_sample;
    return output_sample;
}

void apply_lowpass_filter_to_buffer (float *buffer, int num_samples, float cutoff_freq_hz, float sample_rate_hz)
{
    lowpass_filter_setup(cutoff_freq_hz, sample_rate_hz);
    int i;
    for (i = 0; i < num_samples; i++)
    {
        buffer[i] = lowpass_filter_process(buffer[i]);
    }
}

void apply_dynamic_lowpass_filter (float *buffer, int64_t num_samples, float min_freq, float max_freq, float sample_rate)
{
    int64_t i;
    float envelope = 0.0f;
    float release = 0.9995f; // Very slow release to avoid "pumping" artifacts on LPF
    float attack = 0.1f; 

    // Use a simple envelope follower to modulate LPF cutoff
    for (i = 0; i < num_samples; i++)
    {
        float abs_sample = fabsf(buffer[i]);
        
        // Normalize sample roughly to 0..1 range for envelope calculation (assuming 32767 peak)
        float norm_sample = abs_sample / 32767.0f;

        if (norm_sample > envelope)
            envelope = envelope + attack * (norm_sample - envelope);
        else
            envelope = envelope * release;

        // Map envelope (0..1) to frequency (max_freq .. min_freq)
        // Higher envelope (louder) -> Lower frequency (more filtering)
        float target_freq = max_freq - (envelope * (max_freq - min_freq));
        
        // Clamp
        if (target_freq < min_freq) target_freq = min_freq;
        if (target_freq > max_freq) target_freq = max_freq;

        // Update filter alpha for this sample
        double rc = 1.0 / (2.0 * M_PI * target_freq);
        double dt = 1.0 / sample_rate;
        lp_alpha = (float)(dt / (rc + dt));

        // Apply filter
        buffer[i] = lowpass_filter_process(buffer[i]);
    }
}

void apply_slew_limiter (float *buffer, int64_t num_samples, float slew_factor)
{
    int64_t i;
    float max_delta = 32767.0f * slew_factor;
    
    // Process forward
    for (i = 1; i < num_samples; i++)
    {
        float delta = buffer[i] - buffer[i-1];
        if (delta > max_delta)
        {
            buffer[i] = buffer[i-1] + max_delta;
        }
        else if (delta < -max_delta)
        {
            buffer[i] = buffer[i-1] - max_delta;
        }
    }
}

void apply_mid_scoop (float *buffer, int64_t num_samples, float db_cut, float sample_rate)
{
    // Standard Peaking EQ Biquad implementation
    // Center freq: 1000Hz
    // Q: 1.0 (Wide bandwidth)
    float f0 = 1000.0f;
    float Q = 1.0f;
    float gain = -db_cut; // User supplies positive dB for cut, so invert it.
    
    float A = powf(10.0f, gain / 40.0f);
    float w0 = 2.0f * M_PI * f0 / sample_rate;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);

    Biquad bq;
    
    float b0 = 1.0f + alpha * A;
    float b1 = -2.0f * cos_w0;
    float b2 = 1.0f - alpha * A;
    float a0 = 1.0f + alpha / A;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha / A;

    // Normalize coefficients
    bq.b0 = b0 / a0;
    bq.b1 = b1 / a0;
    bq.b2 = b2 / a0;
    bq.a1 = a1 / a0;
    bq.a2 = a2 / a0;
    
    bq.z1 = 0.0f;
    bq.z2 = 0.0f;

    int64_t i;
    for (i = 0; i < num_samples; i++)
    {
        float in = buffer[i];
        float out = bq.b0 * in + bq.z1;
        bq.z1 = bq.b1 * in + bq.z2 - bq.a1 * out;
        bq.z2 = bq.b2 * in - bq.a2 * out;
        buffer[i] = out;
    }
}

void apply_compressor (float *buffer, int64_t num_samples, float ratio, float threshold_db, float sample_rate)
{
    // Simple Feed-Forward Compressor
    // Threshold: threshold_db relative to 32767
    // Attack: 5ms
    // Release: 100ms
    
    float threshold_linear = powf(10.0f, threshold_db / 20.0f) * 32767.0f;
    float envelope = 0.0f;
    float attack_time = 0.005f; 
    float release_time = 0.100f; 
    
    float ga = expf(-1.0f / (sample_rate * attack_time));
    float gr = expf(-1.0f / (sample_rate * release_time));

    int64_t i;
    for (i = 0; i < num_samples; i++)
    {
        float abs_sample = fabsf(buffer[i]);
        
        // Envelope Follower
        if (abs_sample > envelope) 
            envelope = ga * envelope + (1.0f - ga) * abs_sample;
        else 
            envelope = gr * envelope + (1.0f - gr) * abs_sample;

        // Gain Reduction
        if (envelope > threshold_linear) {
            // Convert envelope to dB relative to threshold
            float over_db = 20.0f * log10f(envelope / threshold_linear);
            
            // Calculate gain reduction in dB
            float gain_reduction_db = over_db * (1.0f - 1.0f/ratio);
            
            // Convert to linear gain
            float gain = powf(10.0f, -gain_reduction_db / 20.0f);
            
            buffer[i] *= gain;
        }
    }
    
    // Normalize back to peak to maximize bit usage
    normalize_buffer_to_peak(buffer, num_samples, 32767.0f);
}

void normalize_buffer_to_peak (float *buffer, int64_t num_samples, float target_peak)
{
    int64_t i;
    float current_peak = 0.0f;
    
    for (i = 0; i < num_samples; i++) {
        if (fabsf(buffer[i]) > current_peak) current_peak = fabsf(buffer[i]);
    }
    
    if (current_peak > 0.0f) {
        float gain = target_peak / current_peak;
        for (i = 0; i < num_samples; i++) buffer[i] *= gain;
    }
}


void apply_soft_clip_to_buffer (float *buffer, int64_t num_samples)
{
    int64_t i;
    float peak = 0.0f;
    float drive = 1.5f; // Slight drive to force saturation

    // Apply tanh with drive
    for (i = 0; i < num_samples; i++)
    {
        // Normalize to -1..1 range, apply drive, then tanh
        float s = buffer[i] / 32767.0f;
        s = tanhf(s * drive);
        buffer[i] = s * 32767.0f;

        if (fabsf(buffer[i]) > peak)
            peak = fabsf(buffer[i]);
    }

    // Re-normalize to maximize dynamic range after clipping
    if (peak > 0.0f)
    {
        float makeup_gain = 32767.0f / peak;
        for (i = 0; i < num_samples; i++)
        {
            buffer[i] *= makeup_gain;
        }
    }
}

float returnMedianValue (float a, float b, float c, float d, float e)
{
    float sorted_arr[5];
    sorted_arr[0] = a;
    sorted_arr[1] = b;
    sorted_arr[2] = c;
    sorted_arr[3] = d;
    sorted_arr[4] = e;
    int i, j;
    float key;
    for (i = 1; i < 5; i++)
    {
	key = sorted_arr[i];
	j = i - 1;
	while (j >= 0 && sorted_arr[j] > key)
	{
	    sorted_arr[j + 1] = sorted_arr[j];
	    j = j - 1;
	}
	sorted_arr[j + 1] = key;
    }
    return sorted_arr[2];	// Median is the middle element (index 2 for 5 elements)
}

void save_all (void)
{
    save_encoder_state ();
    save_decoder_state ();
}

void restore_all (void)
{
    restore_encoder_state ();
    restore_decoder_state ();
}


void save_encoder_state (void)
{
    save_encoder_step_size = encoder_step_size;
    save_encoder_history = encoder_history;
}

void save_decoder_state (void)
{
    save_decoder_step_size = decoder_step_size;
    save_decoder_history = decoder_history;
}

void restore_encoder_state (void)
{
    encoder_step_size = save_encoder_step_size;
    encoder_history = save_encoder_history;
}

void restore_decoder_state (void)
{
    decoder_step_size = save_decoder_step_size;
    decoder_history = save_decoder_history;
}


void usage (char *programname)
{
    fprintf (stderr, "%s %s %s\n", PROGNAME, __DATE__, __TIME__);
    fprintf (stderr, "Usage: %s -i INPUTFILE -o OUTFILE [-r RATE | -R] [options]\n", programname);
    fprintf (stderr, "\n");
    fprintf (stderr, "    Options for specifying input and output format details.\n");
    fprintf (stderr, "       -i specifies the input file. (WAV, MP3, OGG, etc.)\n");
    fprintf (stderr, "       -o specifies the output file name.\n");
    fprintf (stderr, "       -r specifies the output bitrate in samples/second. (Default: %ld)\n", TARGETRATE);
    fprintf (stderr, "       -R uses the input sample bitrate as the encoding rate.\n");
    fprintf (stderr, "\n    Pre-processing Options:\n");
    fprintf (stderr, "       -p <PRESET> Load a preset configuration (music, deep, speech).\n");
    fprintf (stderr, "       -C <RATIO>  Compressor. Limits dynamic range with RATIO (e.g. 4.0).\n");
    fprintf (stderr, "       -M <DB>     Mid-Range Scoop. Cuts 1kHz by DB amount (e.g. 6.0).\n");
    fprintf (stderr, "       -V <FACTOR> Vocal Priority. Penalizes drift to keep vocals clear (e.g. 5.0).\n");
    fprintf (stderr, "       -s          Enable Soft Clipping (Tanh Limiter).\n");
    fprintf (stderr, "       -l <FREQ>   Static Low-Pass Filter at FREQ (Hz).\n");
    fprintf (stderr, "       -D <MIN,MAX> Dynamic Low-Pass Filter. (e.g. 3000,12000)\n");
    fprintf (stderr, "       -S <FACTOR> Slew Rate Limiter. (e.g. 0.20)\n");
    fprintf (stderr, "\n    Encoder Options:\n");
    fprintf (stderr, "       -L <DEPTH>  Lookahead Depth (1-%d). Default 3. Higher is slower but better.\n", MAX_LOOKAHEAD_DEPTH);
    fprintf (stderr, "\n");
}
