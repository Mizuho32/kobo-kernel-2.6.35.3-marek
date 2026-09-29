/*
 * This kernel tree (2.6.35.3) only ships compiler-gcc.h dispatch headers up
 * to gcc4 (linux/compiler-gcc4.h), but we're cross-compiling this
 * out-of-tree module with a much newer host toolchain (gcc 8.3, see
 * mds/wifi-hostsleep/). compiler-gcc4.h doesn't actually branch on
 * __GNUC__/__GNUC_MINOR__ except to reject gcc 4.1.x specifically, so
 * reusing it for gcc8 is safe. This file only needs to exist so
 * compiler-gcc.h's `#include gcc_header(__GNUC__)` can find it; the real
 * kernel include dir (searched after this wrap/ dir, see Makefile) still
 * provides the actual compiler-gcc4.h contents included below.
 */
#include <linux/compiler-gcc4.h>
