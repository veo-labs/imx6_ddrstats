/*
 * Copyright (c) 2012 Philipp Zabel
 * based on omap4_ddrstat.c,
 * Copyright (c) 2010 Mans Rullgard
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

#define PAGE_SIZE 4096

#define MMDC0_BASE 0x021b0000
#define MMDC1_BASE 0x021b4000

#define MMDC_MADPCR0 0x0410
#define MMDC_MADPCR1 0x0414
#define MMDC_MADPSR0 0x0418	/* total cycles */
#define MMDC_MADPSR1 0x041c	/* busy cycles */
#define MMDC_MADPSR2 0x0420	/* total read accesses */
#define MMDC_MADPSR3 0x0424	/* total write accesses */
#define MMDC_MADPSR4 0x0428	/* total read bytes */
#define MMDC_MADPSR5 0x042c	/* total write bytes */

#define MADPCR0_DBG_EN	(1 << 0)
#define MADPCR0_DBG_RST	(1 << 1)
#define MADPCR0_PRF_FRZ	(1 << 2)
#define MADPCR0_CYC_OVF	(1 << 3)

#define MADPCR1_PRF_AXI_ID_SHIFT	0	/* profiling AXI ID */
#define MADPCR1_PRF_AXI_ID_MASK_SHIFT	16	/* profiling AXI ID mask */

/*
 * AXI IDs that match
 * (AXI-ID & PRF_AXI_ID_MASK) Xnor (PRF_AXI_ID & PRF_AXI_ID_MASK)
 * are taken for profiling
 *
 * To monitor AXI ID's between A100 till A1FF, use
 * - PRF_AXI_ID= 0xa100
 * - PRF_AXI_ID_MASK = 0xff00
 */

static void *mmdc0, *mmdc1;

struct mmdc_stats {
	uint32_t cycles;
	uint32_t busy_cycles;
	uint32_t read_accesses;
	uint32_t write_accesses;
	uint32_t read_bytes;
	uint32_t write_bytes;
};

static struct mmdc_stats mmdc0_end;
static struct mmdc_stats mmdc1_end;

static unsigned short axi_id;
static unsigned short axi_id_mask;
static bool pretty;

static void *mmdc_init(int fd, unsigned base)
{
	void *mem = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd,
			 base);
	volatile uint32_t *mmdc = mem;

	if (mem == MAP_FAILED)
		return NULL;

	mmdc[MMDC_MADPCR0 >> 2] = 0;
	/* assert DBG_RST, write 1 to clear CYC_OVF */
	mmdc[MMDC_MADPCR0 >> 2] = MADPCR0_DBG_RST | MADPCR0_CYC_OVF;
	/* deassert DBG_RST, enable DBG_EN and set PRF_FRZ */
	mmdc[MMDC_MADPCR0 >> 2] = MADPCR0_DBG_EN | MADPCR0_PRF_FRZ;

	mmdc[MMDC_MADPCR1 >> 2] = (axi_id_mask << MADPCR1_PRF_AXI_ID_MASK_SHIFT)
				| (axi_id << MADPCR1_PRF_AXI_ID_SHIFT);

	return mem;
}

static void mmdc_read(volatile uint32_t *mmdc, struct mmdc_stats *st)
{
	st->cycles         = mmdc[MMDC_MADPSR0 >> 2];
	st->busy_cycles    = mmdc[MMDC_MADPSR1 >> 2];
	st->read_accesses  = mmdc[MMDC_MADPSR2 >> 2];
	st->write_accesses = mmdc[MMDC_MADPSR3 >> 2];
	st->read_bytes     = mmdc[MMDC_MADPSR4 >> 2];
	st->write_bytes    = mmdc[MMDC_MADPSR5 >> 2];
}

static void mmdc_print_pretty(const char *tag, struct mmdc_stats *st)
{
	static const char * const unit[] = { "B", "KiB", "MiB", "GiB" };
	unsigned long read_size = 0, write_size = 0;
	unsigned long read_count = st->read_bytes;
	unsigned long write_count = st->write_bytes;
	int read_unit = 0, write_unit = 0;

	if (st->read_accesses)
		read_size = (read_count + st->read_accesses - 1) /
			    st->read_accesses;
	if (st->write_accesses)
		write_size = (write_count + st->write_accesses - 1) /
			     st->write_accesses;

	while (read_count > 1023 && read_unit < 3) {
		read_count /= 1024;
		read_unit++;
	}

	while (write_count > 1023 && write_unit < 3) {
		write_count /= 1024;
		write_unit++;
	}

	printf("%s %.2f%% busy %lu %s reads (%lu B / access) %lu %s writes (%lu B / access)",
	       tag, (double)100.0 * st->busy_cycles / st->cycles,
	       read_count, unit[read_unit], read_size,
	       write_count, unit[write_unit], write_size);
}

static void mmdc_print(const char *tag, struct mmdc_stats *st)
{
	if (pretty)
		mmdc_print_pretty(tag, st);
	else
		printf("%s %.2f%% busy %u reads (%u bytes) %u writes (%u bytes)",
		       tag, (double)100.0 * st->busy_cycles / st->cycles,
		       st->read_accesses, st->read_bytes,
		       st->write_accesses, st->write_bytes);
}

static int perf_init(void)
{
	int fd = open("/dev/mem", O_RDWR);
	int err = 0;

	if (fd == -1)
		return -1;

	mmdc0 = mmdc_init(fd, MMDC0_BASE);
	mmdc1 = mmdc_init(fd, MMDC1_BASE);

	if (!mmdc0 || !mmdc1)
		err = -1;

	close(fd);
	return err;
}

static void perf_start(void)
{
	volatile uint32_t *mmdc = mmdc0;

	if (mmdc) {
		/* Assert reset, clear overflow flag */
		mmdc[MMDC_MADPCR0 >> 2] |= MADPCR0_DBG_RST | MADPCR0_CYC_OVF;
		mmdc[MMDC_MADPCR0 >> 2] &= ~(MADPCR0_DBG_RST | MADPCR0_PRF_FRZ);
	}
	mmdc = mmdc1;
	if (mmdc) {
		/* Assert reset, clear overflow flag */
		mmdc[MMDC_MADPCR0>>2] |= MADPCR0_DBG_RST | MADPCR0_CYC_OVF;
		mmdc[MMDC_MADPCR0>>2] &= ~(MADPCR0_DBG_RST | MADPCR0_PRF_FRZ);
	}
}

static void perf_stop(void)
{
	volatile uint32_t *mmdc = mmdc0;

	if (mmdc) {
		mmdc[MMDC_MADPCR0>>2] |= MADPCR0_PRF_FRZ;
		if (mmdc[MMDC_MADPCR0>>2] & MADPCR0_CYC_OVF)
			printf("overflow 0!\n");
		mmdc_read(mmdc, &mmdc0_end);
	}
	mmdc = mmdc1;
	if (mmdc) {
		mmdc[MMDC_MADPCR0>>2] |= MADPCR0_PRF_FRZ;
		if (mmdc[MMDC_MADPCR0>>2] & MADPCR0_CYC_OVF)
			printf("overflow 1!\n");
		mmdc_read(mmdc, &mmdc1_end);
	}
}

static void perf_print(void)
{
	if (mmdc0) {
		mmdc_print("MMDC0", &mmdc0_end);
		if (mmdc1_end.cycles) {
			printf("\t");
			mmdc_print("MMDC1", &mmdc1_end);
		}
		printf("\n");
	}
}

static void perf_close(void)
{
	if (mmdc0)
		munmap(mmdc0, PAGE_SIZE);
	if (mmdc1)
		munmap(mmdc1, PAGE_SIZE);
}

struct axi_filter {
	char *name;
	unsigned short axi_id_mask;
	unsigned short axi_id;
};

/* Table 43-8. i.MX 6Dual/6Quad AXI ID */
static struct axi_filter filters[] = {
	{ "arm-s0",    0b11100000000111, 0b00000000000000 },
	{ "arm-s1",    0b11100000000111, 0b00000000000001 },
	{ "ipu1",      0b11111111100111, 0b00000000000100 },
	{ "ipu1-0",    0b11111111111111, 0b00000000000100 },
	{ "ipu1-1",    0b11111111111111, 0b00000000001100 },
	{ "ipu1-2",    0b11111111111111, 0b00000000010100 },
	{ "ipu1-3",    0b11111111111111, 0b00000000011100 },
	{ "ipu2",      0b11111111100111, 0b00000000000101 },
	{ "ipu2-0",    0b11111111111111, 0b00000000000101 },
	{ "ipu2-1",    0b11111111111111, 0b00000000001101 },
	{ "ipu2-2",    0b11111111111111, 0b00000000010101 },
	{ "ipu2-3",    0b11111111111111, 0b00000000011101 },
	{ "gpu3d-a",   0b11110000111111, 0b00000000000010 },
	{ "gpu2d-a",   0b11110000111111, 0b00000000001010 },
	{ "vdoa",      0b11111100111111, 0b00000000010010 },
	{ "openvg",    0b11110000111111, 0b00000000100010 },
	{ "hdmi",      0b11111111111111, 0b00000100011010 },
	{ "sdma-brst", 0b11111111111111, 0b00000101011010 },
	{ "sdma-per",  0b11111111111111, 0b00000110011010 },
	{ "caam",      0b00001111111111, 0b00000000011010 },
	{ "usb",       0b11001111111111, 0b00000001011010 },
	{ "enet",      0b11111111111111, 0b00000010011010 },
	{ "hsi",       0b11111111111111, 0b00000011011010 },
	{ "usdhc1",    0b11111111111111, 0b00000111011010 },
	{ "gpu3d-b",   0b11110000111111, 0b00000000000011 },
	/* the reference manual lists a second gpu3d-b instead of gpu2d-b */
	{ "gpu2d-b",   0b11110000111111, 0b00000000001011 },
	{ "vpu-prime", 0b11110000111111, 0b00000000010011 },
	{ "pcie",      0b11100000111111, 0b00000000011011 },
	{ "dap",       0b11111111111111, 0b00000000100011 },
	{ "apbh-dma",  0b11111111111111, 0b00000010100011 },
	{ "bch40",     0b00001111111111, 0b00000001100011 },
	{ "sata",      0b11111111111111, 0b00000011100011 },
	{ "mlb150",    0b11111111111111, 0b00000100100011 },
	{ "usdhc2",    0b11111111111111, 0b00000101100011 },
	{ "usdhc3",    0b11111111111111, 0b00000110100011 },
	{ "usdhc4",    0b11111111111111, 0b00000111100011 },
	{},
};

void setup_axi_filter(const char *master)
{
	struct axi_filter *filter;

	for (filter = filters; filter->name != NULL; filter++) {
		if (strcmp(filter->name, master) == 0) {
			printf("filtering for AXI IDs from master '%s'\n",
			       filter->name);
			axi_id = filter->axi_id;
			axi_id_mask = filter->axi_id_mask;
			return;
		}
	}

	printf("not filtering for AXI IDs. Possible AXI masters:\n ");
	for (filter = filters; filter->name != NULL; filter++)
		printf(" %s", filter->name);
	printf("\n");
	axi_id = 0;
	axi_id_mask = 0;
}

int main(int argc, char **argv)
{
	int delay = 1;
	char *endp;

	if (argc > 1 && strcmp(argv[1], "--help") == 0) {
		struct axi_filter *filter;

		printf("Usage: imx6_ddrstat [-h] [interval] [filter]\n"
		       "  -h		output in human readable format\n"
		       " interval:	1-4 seconds\n"
		       " possible AXI master filters:\n ");
		for (filter = filters; filter->name != NULL; filter++)
			printf(" %s", filter->name);
		printf("\n");
		return 0;
	}
	if (argc > 1 && strcmp(argv[1], "-h") == 0) {
		pretty = true;
		argv++;
		argc--;
	}
	if (argc > 1) {
		delay = strtol(argv[1], &endp, 0);
		if (delay > 4)
			return 1;
		if (endp == argv[1] && argc == 2)
			setup_axi_filter(argv[1]);
	}
	if (argc > 2)
		setup_axi_filter(argv[2]);

	if (delay <= 0)
		delay = 1;
	printf("interval %d s\n", delay);

	if (perf_init())
		return 1;

	for (;;) {
		perf_start();
		sleep(delay);
		perf_stop();
		perf_print();
	}

	perf_close();
	return 0;
}
