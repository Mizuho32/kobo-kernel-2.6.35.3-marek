obj-m += dhd_hostsleep_hook.o

# This kernel tree only ships compiler-gccN.h dispatch headers up to gcc4;
# wrap/linux/compiler-gcc8.h forwards to compiler-gcc4.h so a newer host
# toolchain (e.g. gcc 8.3) can still cross-compile this out-of-tree module.
# See wrap/linux/compiler-gcc8.h for why this is safe.
ccflags-y += -I$(src)/wrap
