//------------------------------------------------------------------------------------------------------------
//
// I2C ADC LTC2309 (12bits-8CH) Test Application. (Use 8 Single-Ended, Unipolar Mode)
//
// compile : make
//
// Run : sudo ./<created excute file name>
//
//------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <getopt.h>

#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <ctype.h>

#include "i2c.h"

//------------------------------------------------------------------------------------------------------------
#if defined(__DEBUG__)
	#define	dbg_msg(fmt, args...)	fprintf(stderr,"[%s/%d/%s] " fmt, __FILE__, __LINE__, __func__, ##args)
#else
	#define	dbg_msg(fmt, args...)
#endif

#define info_msg(fmt, args...)	fprintf(stdout, "[INFO] " fmt, ##args)
#define	err_msg(fmt, args...)	fprintf(stdout, "[*ERROR*] " fmt, ##args)
#define msg(fmt, args...)		fprintf(stdout, fmt, ##args)

//------------------------------------------------------------------------------------------------------------
// LTC2309 DEVICE ADDR
//------------------------------------------------------------------------------------------------------------
// LTC2309 Chip Pin Status (AD1, AD0)
//
// 0x08 = (  LOW,   LOW), 0x09 = (  LOW, FLOAT), 0x0A = ( LOW, HIGH), 0x0B = (FLOAT,  HIGH), 
// 0x18 = (FLOAT, FLOAT), 0x19 = (FLOAT,   LOW), 0x1A = (HIGH,  LOW), 0x1B = ( HIGH, FLOAT), 
// 0x14 = ( HIGH,  HIGH)
//
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
// LTC2309 Reg Bits
//------------------------------------------------------------------------------------------------------------
//
//  BIT7  BIT6  BIT5  BIT4  BIT3  BIT2  BIT1  BIT0
//  S/D   O/S   S1    S0    UNI    SLP   X     X
//
//  S/D : Single-ended/Defferential
//  O/S : ODD/Sign
//  S1  : Channel Select 1
//  S0  : Channel Select 0
//  UNI : UNIPOLAR/BIPOLAR
//  SLP : Sleep Mode
//
//  if ) Channel 0 Read
//       Single-ended = 1 | Sign = 0 | S1 = 0 | S0 = 0 | Unipolar = 1 | sleep = 0 | x | x | = 0x88
//  if ) Channel 1 Read
//       Single-ended = 1 | ODD = 1  | S1 = 0 | S0 = 0 | Unipolar = 1 | sleep = 0 | x | x | = 0xC8
//  if ) Channel 2 Read
//       Single-ended = 1 | Sign = 0 | S1 = 0 | S0 = 1 | Unipolar = 1 | sleep = 0 | x | x | = 0xA8
//  if ) Channel 3 Read
//       Single-ended = 1 | ODD = 1  | S1 = 0 | S0 = 1 | Unipolar = 1 | sleep = 0 | x | x | = 0xD8
//
//------------------------------------------------------------------------------------------------------------
#define SWAP_WORD(x)	(((x >> 8) & 0xFF) | ((x << 8) & 0xFF00))

#define	ADC_REF_VOLTAGE	5

// Reference 5V, ADC weight value = 1200uV
#define ADC_WEIGHT_uV	((ADC_REF_VOLTAGE * 1000000) / 4096)

enum {
	CHIP_ADC0 = 0,	
	CHIP_ADC1	,
	CHIP_ADC2	,
	CHIP_ADC3	,
	CHIP_ADC4	,
	CHIP_ADC5	,
	CHIP_ADC_CNT,
	NOT_USED	,	
};

const unsigned char ADC_ADDR[] = {
	0x08, 0x09, 0x0A, 0x0B, 0x18, 0x19
};

const unsigned char ADC_CH[] = {
	0x88, 0xC8, 0x98, 0xD8, 0xA8, 0xE8, 0xB8, 0xF8
};

struct pin_info {
	const char *name;
	unsigned char pin_num;
	unsigned char adc_idx;
	unsigned char ch_idx;
};

const struct pin_info HEADER_CON1[] = {
	{ "CON1.0" ,   0, NOT_USED , 0},	// Header Pin 0

	{ "CON1.1" ,   1, CHIP_ADC0, 0},	// Header Pin 1 Info
	{ "CON1.2" ,   2, CHIP_ADC0, 1},	// Header Pin 2 Info
	{ "CON1.3" ,   3, CHIP_ADC1, 0},
	{ "CON1.4" ,   4, CHIP_ADC0, 2},
	{ "CON1.5" ,   5, CHIP_ADC1, 1},
	{ "CON1.6" ,   6, NOT_USED , 0},
	{ "CON1.7" ,   7, CHIP_ADC1, 2},
	{ "CON1.8" ,   8, CHIP_ADC1, 3},
	{ "CON1.9" ,   9, NOT_USED , 0},
	{ "CON1.10",  10, CHIP_ADC1, 4},

	{ "CON1.11",  11, CHIP_ADC1, 5},
	{ "CON1.12",  12, CHIP_ADC1, 6},
	{ "CON1.13",  13, CHIP_ADC1, 7},
	{ "CON1.14",  14, NOT_USED , 0},
	{ "CON1.15",  15, CHIP_ADC2, 0},
	{ "CON1.16",  16, CHIP_ADC2, 1},
	{ "CON1.17",  17, CHIP_ADC0, 3},
	{ "CON1.18",  18, CHIP_ADC2, 2},
	{ "CON1.19",  19, CHIP_ADC2, 3},
	{ "CON1.20",  20, NOT_USED , 0},

	{ "CON1.21",  21, CHIP_ADC2, 4},
	{ "CON1.22",  22, CHIP_ADC2, 5},
	{ "CON1.23",  23, CHIP_ADC2, 6},
	{ "CON1.24",  24, CHIP_ADC2, 7},
	{ "CON1.25",  25, NOT_USED , 0},
	{ "CON1.26",  26, CHIP_ADC3, 0},
	{ "CON1.27",  27, CHIP_ADC3, 1},
	{ "CON1.28",  28, CHIP_ADC3, 2},
	{ "CON1.29",  29, CHIP_ADC3, 3},
	{ "CON1.30",  30, NOT_USED , 0},

	{ "CON1.31",  31, CHIP_ADC3, 4},
	{ "CON1.32",  32, CHIP_ADC3, 5},
	{ "CON1.33",  33, CHIP_ADC3, 6},
	{ "CON1.34",  34, NOT_USED , 0},
	{ "CON1.35",  35, CHIP_ADC3, 7},
	{ "CON1.36",  36, CHIP_ADC4, 0},
	{ "CON1.37",  37, NOT_USED , 0},
	{ "CON1.38",  38, CHIP_ADC0, 4},
	{ "CON1.39",  39, NOT_USED , 0},
	{ "CON1.40",  40, NOT_USED , 0},
};

const struct pin_info HEADER_P3[] = {
	{ "P3.0" ,  0, NOT_USED, 0},	// Header Pin 0
	{ "P3.1" ,  1, NOT_USED, 0},	// Header Pin 1 Info
	{ "P3.2" ,  2, CHIP_ADC5, 0},	// Header Pin 2 Info
	{ "P3.3" ,  3, CHIP_ADC5, 1},
	{ "P3.4" ,  4, NOT_USED, 0},
	{ "P3.5" ,  5, CHIP_ADC5, 2},
	{ "P3.6" ,  6, CHIP_ADC5, 3},
	{ "P3.7" ,  7, NOT_USED, 0},
	{ "P3.8" ,  8, CHIP_ADC5, 4},
	{ "P3.9" ,  9, CHIP_ADC5, 5},
	{ "P3.10", 10, NOT_USED, 0},
};

const struct pin_info HEADER_P13[] = {
	{ "P13.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P13.1", 1, NOT_USED , 0},	// Header Pin 1 Info
	{ "P13.2", 2, CHIP_ADC4, 1},	// Header Pin 2 Info
	{ "P13.3", 3, CHIP_ADC0, 5},
	{ "P13.4", 4, CHIP_ADC4, 2},
	{ "P13.5", 5, CHIP_ADC4, 3},
	{ "P13.6", 6, CHIP_ADC4, 4},
	{ "P13.7", 7, CHIP_ADC4, 5},
};

const struct pin_info HEADER_P1_1[] = {
	{ "P1-1.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P1-1.1", 1, CHIP_ADC0, 0},	// Header Pin 1 Info
	{ "P1-1.2", 2, CHIP_ADC0, 1},	// Header Pin 2 Info
	{ "P1-1.3", 3, CHIP_ADC0, 2},
	{ "P1-1.4", 4, CHIP_ADC0, 3},
	{ "P1-1.5", 5, CHIP_ADC0, 4},
	{ "P1-1.6", 6, CHIP_ADC0, 5},
	{ "P1-1.7", 7, CHIP_ADC0, 6},
	{ "P1-1.8", 8, CHIP_ADC0, 7},
};

const struct pin_info HEADER_P1_2[] = {
	{ "P1-2.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P1-2.1", 1, CHIP_ADC1, 0},	// Header Pin 1 Info
	{ "P1-2.2", 2, CHIP_ADC1, 1},	// Header Pin 2 Info
	{ "P1-2.3", 3, CHIP_ADC1, 2},
	{ "P1-2.4", 4, CHIP_ADC1, 3},
	{ "P1-2.5", 5, CHIP_ADC1, 4},
	{ "P1-2.6", 6, CHIP_ADC1, 5},
	{ "P1-2.7", 7, CHIP_ADC1, 6},
	{ "P1-2.8", 8, CHIP_ADC1, 7},
};

const struct pin_info HEADER_P1_3[] = {
	{ "P1-3.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P1-3.1", 1, CHIP_ADC2, 0},	// Header Pin 1 Info
	{ "P1-3.2", 2, CHIP_ADC2, 1},	// Header Pin 2 Info
	{ "P1-3.3", 3, CHIP_ADC2, 2},
	{ "P1-3.4", 4, CHIP_ADC2, 3},
	{ "P1-3.5", 5, CHIP_ADC2, 4},
	{ "P1-3.6", 6, CHIP_ADC2, 5},
	{ "P1-3.7", 7, CHIP_ADC2, 6},
	{ "P1-3.8", 8, CHIP_ADC2, 7},
};

const struct pin_info HEADER_P1_4[] = {
	{ "P1-4.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P1-4.1", 1, CHIP_ADC3, 0},	// Header Pin 1 Info
	{ "P1-4.2", 2, CHIP_ADC3, 1},	// Header Pin 2 Info
	{ "P1-4.3", 3, CHIP_ADC3, 2},
	{ "P1-4.4", 4, CHIP_ADC3, 3},
	{ "P1-4.5", 5, CHIP_ADC3, 4},
	{ "P1-4.6", 6, CHIP_ADC3, 5},
	{ "P1-4.7", 7, CHIP_ADC3, 6},
	{ "P1-4.8", 8, CHIP_ADC3, 7},
}; 

const struct pin_info HEADER_P1_5[] = {
	{ "P1-5.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P1-5.1", 1, CHIP_ADC4, 0},	// Header Pin 1 Info
	{ "P1-5.2", 2, CHIP_ADC4, 1},	// Header Pin 2 Info
	{ "P1-5.3", 3, CHIP_ADC4, 2},
	{ "P1-5.4", 4, CHIP_ADC4, 3},
	{ "P1-5.5", 5, CHIP_ADC4, 4},
	{ "P1-5.6", 6, CHIP_ADC4, 5},
	{ "P1-5.7", 7, CHIP_ADC4, 6},
	{ "P1-5.8", 8, CHIP_ADC4, 7},
};

const struct pin_info HEADER_P1_6[] = {
	{ "P1-6.0", 0, NOT_USED , 0},	// Header Pin 0
	{ "P1-6.1", 1, CHIP_ADC5, 0},	// Header Pin 1 Info
	{ "P1-6.2", 2, CHIP_ADC5, 1},	// Header Pin 2 Info
	{ "P1-6.3", 3, CHIP_ADC5, 2},
	{ "P1-6.4", 4, CHIP_ADC5, 3},
	{ "P1-6.5", 5, CHIP_ADC5, 4},
	{ "P1-6.6", 6, CHIP_ADC5, 5},
	{ "P1-6.7", 7, CHIP_ADC5, 6},
	{ "P1-6.8", 8, CHIP_ADC5, 7},
};

#define	ARRARY_SIZE(x)	(sizeof(x) / sizeof(x[0]))

//------------------------------------------------------------------------------------------------------------
const char *OPT_DEVICE_NAME = "/dev/i2c-0";
const char *OPT_PIN_NAME = '\0';
unsigned char opt_view = 0;
unsigned char opt_iter = 0;
unsigned char opt_unit = 0;

//------------------------------------------------------------------------------------------------------------
int read_pin_value (int fd, struct pin_info *info)
{
	int read_val = 0;

	if (!i2c_set_addr(fd, ADC_ADDR[info->adc_idx])) {
		// Dummy read for chip wake up & conversion
		i2c_read_word(fd, ADC_CH[info->ch_idx]);
		read_val  = i2c_read_word(fd, ADC_CH[info->ch_idx]);
		read_val  = (read_val < 0) ? 0 : read_val;
		read_val  = (SWAP_WORD(read_val) >> 4) & 0xFFF;
		dbg_msg("adc_idx = %d, ch_idx = %d, %d\n", info->adc_idx, info->ch_idx, read_val);
	}
	return read_val;
}

//------------------------------------------------------------------------------------------------------------
unsigned long adc_convert_value (unsigned short adc_value)
{
	unsigned long volt = adc_value * ADC_WEIGHT_uV;

	return	(opt_unit) ? (volt) : (volt / 1000);
}

//------------------------------------------------------------------------------------------------------------
void toupper_str(char *str)
{
	char *p = str, cnt = 0;
	while(*p) {
		*p = toupper((unsigned char )*p);
		p++;
		if (cnt++ > 10)
			break;
	}
}

//------------------------------------------------------------------------------------------------------------
void tolower_str(char *str)
{
	char *p = str, cnt = 0;
	while(*p) {
		*p = tolower((unsigned char )*p);
		p++;
		if (cnt++ > 10)
			break;
	}
}

//------------------------------------------------------------------------------------------------------------
struct pin_info *header_info(const char *h_name, char pin_no, char *p_cnt)
{
	struct pin_info *p;

	if 			(!strncmp("CON1", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_CON1) ? pin_no : 0;
		p 		= pin_no ? &HEADER_CON1[pin_no] : &HEADER_CON1[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_CON1) -1;
	} else if 	(!strncmp("P3"  , h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P3) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P3[pin_no] : &HEADER_P3[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P3) -1;
	} else if	(!strncmp("P13" , h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P13) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P13[pin_no] : &HEADER_P13[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P13) -1;
	} else if	(!strncmp("P1_1", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P1_1) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P1_1[pin_no] : &HEADER_P1_1[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P1_1) -1;
	} else if	(!strncmp("P1_2", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P1_2) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P1_2[pin_no] : &HEADER_P1_2[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P1_2) -1;
	} else if	(!strncmp("P1_3", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P1_3) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P1_3[pin_no] : &HEADER_P1_3[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P1_3) -1;
	} else if	(!strncmp("P1_4", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P1_4) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P1_4[pin_no] : &HEADER_P1_4[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P1_4) -1;
	} else if	(!strncmp("P1_5", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P1_5) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P1_5[pin_no] : &HEADER_P1_5[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P1_5) -1;
	} else if	(!strncmp("P1_6", h_name, sizeof(h_name))) {
		pin_no 	= pin_no < ARRARY_SIZE(HEADER_P1_6) ? pin_no : 0;
		p 		= pin_no ? &HEADER_P1_6[pin_no] : &HEADER_P1_6[1];
		*p_cnt	= pin_no ? 1 : ARRARY_SIZE(HEADER_P1_6) -1;
	} else {
		p = &HEADER_CON1[0];
		*p_cnt	= 0;
	}
	return p;
}

//------------------------------------------------------------------------------------------------------------
int print_pin_info(int fd, const char *name)
{
	char *h_name, *pin_str;
	char str[10], pin_no = 0, pin_cnt = 0, i, iter;
	unsigned long real, avr, max, min, diff;
	struct pin_info *p;

	if (!name)
		return -1;

	memset(str, 0x00, sizeof(str));
	memcpy(str, name, sizeof(name));

	if (h_name  = strtok(str, "."))
		toupper_str(h_name);

	if (pin_str = strtok(NULL, " "))
		pin_no = atoi(pin_str);

	p = header_info(h_name, pin_no, &pin_cnt);
	// DEBUG
	dbg_msg("%s : header = %s, pin = %d, pin_cnt = %d\n", name, h_name, pin_no, pin_cnt);

	if (!opt_view) {
		msg ("%10s\t%8s\t%8s\t%8s\t%8s\n", "PIN Name","Avr","Max","Min","Diff","unit");
		msg ("--------------------------------------------------------------------------\n");
	}

	if (pin_cnt) {
		for (i = 0; i < pin_cnt; i++, p++) {
			real = adc_convert_value(read_pin_value(fd, p));
			avr = real, max = real, min = real;

			iter = opt_iter > 0 ? (opt_iter -1) : 0;
			while(iter--) {
				avr += real;
				max = real > max ? real : max;
				min = real < min ? real : min;
				real = adc_convert_value(read_pin_value(fd, p));
			}
			if (opt_iter)
				avr = avr / opt_iter;

			if (opt_view)
				msg("%ld,", avr);
			else
				msg ("%10s\t%8ld\t%8ld\t%8ld\t%8ld\n", p->name, avr, max, min, max - min);

		}
		msg("\n");
	}

	return 0;
}

//------------------------------------------------------------------------------------------------------------
void print_all_info (int fd)
{
	print_pin_info(fd, "CON1");
	print_pin_info(fd, "P3");
	print_pin_info(fd, "P13");
	print_pin_info(fd, "P1_1");
	print_pin_info(fd, "P1_2");
	print_pin_info(fd, "P1_3");
	print_pin_info(fd, "P1_4");
	print_pin_info(fd, "P1_5");
	print_pin_info(fd, "P1_6");
}

//------------------------------------------------------------------------------------------------------------
static void print_usage(const char *prog)
{
	info_msg("Usage: %s [-DsbdlHOLC3vpNR24SI]\n", prog);
	puts("  -D --device   device to use (default /dev/i2c-0)\n"
	     "  -p --pin      pin name or header name (default \'All pins\'. e.g. \"con1/con1.1...\")\n"
	     "  -u --unit     unit of value (default \"mV\". e.g. \"mV/uV\")\n"
	     "  -v --view     show only values with \",\"\n"
	     "  -i --iter     iterations\n"
	);
	exit(1);
}

//------------------------------------------------------------------------------------------------------------
static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  	1, 0, 'D' },
			{ "pin",		1, 0, 'p' },
			{ "unit",		1, 0, 'u' },
			{ "view",		0, 0, 'v' },
			{ "iter",		1, 0, 'i' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:p:u:vi:", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			OPT_DEVICE_NAME = optarg;
			break;
		case 'p':
			OPT_PIN_NAME = optarg;
			break;
		case 'u':
			toupper_str(optarg);
			opt_unit = (!strncmp(optarg, "UV", sizeof(optarg))) ? 1 : 0;
			break;
		case 'v':
			opt_view = 1;
			break;
		case 'i':
			opt_iter = atoi(optarg);
			opt_iter = (opt_iter) > 100 ? 100 : opt_iter;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

//------------------------------------------------------------------------------------------------------------
int check_adc (int fd)
{
	int i, ret = 0;

	for (i = 0; i < ARRARY_SIZE(ADC_ADDR); i++) {
		i2c_set_addr(fd, ADC_ADDR[i]);
		if(i2c_read_word(fd, ADC_ADDR[i]) < 0) {
			info_msg("Not detect %s ADC%d(Device Addr : 0x%02x)\n",
				OPT_DEVICE_NAME, i, ADC_ADDR[i]);
			ret = -1;
		}
	}
	return ret;
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
int main (int argc, char *argv[])
{
	int fd;
	
	parse_opts(argc, argv);
	if ((fd = i2c_open(OPT_DEVICE_NAME)) < 0)
		return -1;

	if (check_adc(fd)) {
		close(fd);
		return -1;
	}

	if (OPT_PIN_NAME)
		print_pin_info(fd, OPT_PIN_NAME);
	else
		print_all_info(fd);

	return 0;
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

