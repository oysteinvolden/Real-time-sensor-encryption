///////////////////////////////////////////
// This implementation of SHA-256 was    //
// placed in the public domain by:       //
//                                       //
// Petter Solnoer - 15/04/2020           //
///////////////////////////////////////////

#ifndef SHA_256_TABLES_H
#define SHA_256_TABLES_H

#include <climits>
#include <fstream>

// Verify that the data types are as expected

#if (UCHAR_MAX != 0xFFU)
#error UCHAR IS NOT 8 BITS
#endif

#if (USHRT_MAX != 0xFFFFU)
#error USHORT IS NOT 16 BITS
#endif

#if (UINT_MAX != 0xFFFFFFFFU)
#error UINT IS NOT 32 BITS
#endif

#if (ULLONG_MAX != 0xFFFFFFFFFFFFFFFFU)
#error ULONGLONG IS NOT 64 BITS
#endif

// Define sizes

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

// Force inline on compilers
#ifdef _MSC_VER
	#define forceinline __forceinline
#elif defined(__GNUC__)
	#define forceinline inline __attribute__((__always_inline__))
#elif defined(__CLANG__)
	#if __has_attribute(__always_inline__)
		#define forceinline inline __attribute__((__always_inline__))
	#else
		#define forceinline inline
	#endif
#else
	#define forceinline inline
#endif


// Define the basic operations
#define ROTR_32(x,n) ( (x >> n) | (x << (32-n)  ) )
#define SHR(x,n) (x >> n)

// Table with SHA-256 constants
static const u32 K[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
	0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
	0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
       	0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
       	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
       	0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
       	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
       	0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
       	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
       	0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
       	0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
       	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
       	0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2 
};

// The initial hash of SHA-256 consists of the following
// 8 words.
static const u32 H0[8] = {
	0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
	0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
};

#endif
