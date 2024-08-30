#ifndef HEATSHRINK_CONFIG_H
#define HEATSHRINK_CONFIG_H

/* Should functionality assuming dynamic allocation be used? */
#ifndef HEATSHRINK_DYNAMIC_ALLOC
#define HEATSHRINK_DYNAMIC_ALLOC 0
#endif

#if HEATSHRINK_DYNAMIC_ALLOC
    /* Optional replacement of malloc/free */
    #define HEATSHRINK_MALLOC(SZ) malloc(SZ)
    #define HEATSHRINK_FREE(P, SZ) free(P)
#else
    /* Required parameters for static configuration */
    /*How large an input buffer to use for the
    decoder. This impacts how much work the decoder can do in a single
    step, and a larger buffer will use more memory. An extremely small
    buffer (say, 1 byte) will add overhead due to lots of suspend/resume
    function calls, but should not change how well data compresses.
    #define HEATSHRINK_STATIC_INPUT_BUFFER_SIZE 1  We do not need decoding*/

    /*The window size determines how far back in the input can be searched for
    repeated patterns. A `window_sz2` of 8 will only use 256 bytes (2^8),
    while a `window_sz2` of 10 will use 1024 bytes (2^10). The latter uses
    more memory, but may also compress more effectively by detecting more
    repetition. WINDOW_BITS currently must be between 4 and 15.*/
    #define HEATSHRINK_STATIC_WINDOW_BITS 10
    /*The lookahead size determines the max length for repeated patterns that
    are found. If the `lookahead_sz2` is 4, a 50-byte run of 'a' characters
    will be represented as several repeated 16-byte patterns (2^4 is 16),
    whereas a larger `lookahead_sz2` may be able to represent it all at
    once. The number of bits used for the lookahead size is fixed, so an
    overly large lookahead size can reduce compression by adding unused
    size bits to small patterns. LOOKAHEAD_BITS currently must be between 3 and the WINDOW_BITS - 1.*/
    #define HEATSHRINK_STATIC_LOOKAHEAD_BITS 4
#endif

/* Turn on logging for debugging. */
#define HEATSHRINK_DEBUGGING_LOGS 0

/* Use indexing for faster compression. (This requires additional space.) */
#define HEATSHRINK_USE_INDEX 1

#endif
