/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port glue for the mcuboot-size measurement. NOT production.
 *
 * newlib-nano's libc_nano.a pulls these in transitively (assert() ->
 * abort() -> _exit(); mbedtls_calloc == calloc -> _sbrk(); etc.) even
 * though this probe never runs. Every -nostartfiles newlib link needs a
 * retarget layer -- the real firmware's is hal/gd32_libc_stubs.c (which
 * only covers _init/_fini because the vendor startup provides the rest);
 * this is the fuller syscall set that a real bare-metal bootloader link
 * needs, sized like a minimal one would be.
 */
#include <sys/stat.h>
#include <sys/types.h>

int _close(int fd)
{
	(void)fd;
	return -1;
}

int _fstat(int fd, struct stat *st)
{
	(void)fd;
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int fd)
{
	(void)fd;
	return 1;
}

int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig;
	return -1;
}

int _getpid(void)
{
	return 1;
}

off_t _lseek(int fd, off_t offset, int whence)
{
	(void)fd;
	(void)offset;
	(void)whence;
	return 0;
}

ssize_t _read(int fd, void *buf, size_t count)
{
	(void)fd;
	(void)buf;
	(void)count;
	return 0;
}

ssize_t _write(int fd, const void *buf, size_t count)
{
	(void)fd;
	(void)buf;
	return (ssize_t)count;
}

void *_sbrk(ptrdiff_t incr)
{
	extern char  _ebss; /* end of .bss, from the linker script */
	static char *heap_end;
	char        *prev;

	if (heap_end == 0) {
		heap_end = &_ebss;
	}
	prev = heap_end;
	heap_end += incr;
	return prev;
}

void _exit(int status)
{
	(void)status;
	for (;;) {
	}
}
