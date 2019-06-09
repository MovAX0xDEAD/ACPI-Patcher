#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <process.h>

#include <sys/farptr.h>
#include <go32.h>
#include <dpmi.h>
#include <bios.h>
#include "ini.h"

#define WRITE 1

#define NOERR           true
#define ERR             false
#define BIOS_ROM_LOW    0xA0000
#define BIOS_ROM_HIGH   0xFFFFF
#define LMB_start       0xE0000
#define LMB_size        (BIOS_ROM_HIGH-LMB_start)

#define INT13_RAM_LOW           0x90000
#define INT13_RANGE_SZ          (BIOS_ROM_LOW-INT13_RAM_LOW)

#define RSDP_V1_LENGTH          20
#define ACPI_PAD                16
#define BUFFERSIZE              (2*1024*1024) // 2M max for one ASL Table
#define E820_CHUNK              128
#define MAXRANGES               64
#define PTR_TO_PTR64(x)         (((unsigned long long)x) & 0x00000000FFFFFFFF)
#define ALIGN(x,y)              ( ((int)x + (y) - 1)&(~((y) - 1)) )
#define MAXIMUM_GRUB4DOS_DRIVE_MAP_SLOTS 8

#define LOG(...)                fprintf(file_log, __VA_ARGS__); fflush(file_log);
#define HEXLOG(...)             hexdump(__VA_ARGS__)
#define HEXLOG_RAW(...)         hexdump_raw(__VA_ARGS__)
////////////////////////////////////////
// Templates for quick access
//
// HEXLOG(buffer,"", 32);
// HEXLOG_RAW(map_ds,buffer,"", 32);
// LOG("mem: %08x \n", value);
// LOG("ACPI Patcher Error: glb_raw_rsdt=%8x is too low\n", raw_base); err_exit(1);}

// FILE *outfile=fopen("debug.txt","wb");
// fwrite(buffer, 1, size, outfile);
// fclose(outfile);
////////////////////////////////////////

typedef struct  _tree {
 char           name[4+2+1]; // "SSDT99 \0"
 void           *ptr;
 uint32_t       size;
 bool           changed;
 bool           allocated;
 uint32_t       rsdt_ptr;
 uint32_t       xsdt_ptr;
 uint32_t       newstore;
 struct _tree   *next;
} tree_t;

typedef struct  _GRUB4DOS_DRIVE_MAP_SLOT {
 uint8_t        from_drive;
 uint8_t        to_drive;       // 0xFF = memdrive
 uint8_t        max_head;
 uint8_t        max_sector:6;
 uint8_t        disable_lba:1;  // bit 6: disable lba
 uint8_t        read_only:1;    // bit 7: read only
 uint16_t       to_cylinder:13; // max cylinder of the TO drive
 uint16_t       from_cdrom:1;   // bit 13: FROM drive is CDROM(with big 2048-byte sector)
 uint16_t       to_cdrom:1;     // bit 14:  TO  drive is CDROM(with big 2048-byte sector)
 uint16_t       to_lba:1;       // bit 15:  TO  drive support LBA
 uint8_t        to_head;        // max head of the TO drive
 uint8_t        to_sector:6;    // max sector of the TO drive
 uint8_t        fake_write:1;   // bit 6: fake-write or safe-boot
 uint8_t        in_situ:1;      // bit 7: in-situ
 uint64_t       start_sector;
 uint64_t       sector_count;
} __attribute__ ((packed)) GRUB4DOS_DRIVE_MAP_SLOT;

typedef struct  acpi_rsdp_v10 {
 char           signature[8];
 uint8_t        checksum;
 char           oemid[6];
 uint8_t        revision;
 uint32_t       rsdt_address;
} __attribute__ ((packed)) acpi_rsdp_v10_t;

typedef struct  acpi_rsdp_v20 {
 acpi_rsdp_v10_t rsdpv1;
 uint32_t        length;	        /* total length of RSDP (including extended part) */
 uint64_t        xsdt_xaddr;     /* physical address of XSDT */
 uint8_t         ext_checksum;   /* chechsum of whole table */
 uint8_t         reserved[3];
}  __attribute__ ((packed)) acpi_rsdp_v20_t;

typedef struct  acpi_table_header {
 char           signature [4];          /* ACPI signature (4 ASCII characters) */\
 uint32_t       length;	                /* Length of table, in bytes, including header */\
 uint8_t        revision;               /* ACPI Specification minor version # */\
 uint8_t        checksum;               /* To make sum of entire table == 0 */\
 char           oem_id [6];             /* OEM identification */\
 char           oem_table_id [8];       /* OEM table identification */\
 uint32_t       oem_revision;           /* OEM revision number */\
 char           asl_compiler_id [4];    /* ASL compiler vendor ID */\
 uint32_t       asl_compiler_revision;  /* ASL compiler revision number */
} __attribute__ ((packed)) acpi_header_t;

typedef struct  acpi_rsdt {
 acpi_header_t  header;
 acpi_header_t* entry[1];
} __attribute__ ((packed)) acpi_rsdt_t;

typedef struct  acpi_xsdt {
 acpi_header_t  header;
 uint64_t       entry[1];
} __attribute__ ((packed)) acpi_xsdt_t;

typedef struct  acpi_fadt {
 acpi_header_t   header;
 acpi_header_t*  facs_addr;
 acpi_header_t*  dsdt_addr;
 char            somefields1[88];
 uint64_t        facs_xaddr;
 uint64_t        dsdt_xaddr;
 char            somefields2[96];
} __attribute__ ((packed)) acpi_fadt_t;

typedef struct _e820data {
 uint64_t       base;
 uint64_t       len;
 uint32_t       type;
 uint32_t       extattr;
} __attribute__ ((packed)) e820data_t;

typedef struct _e820range {
 uint64_t       start;
 uint64_t       end;
 uint64_t       len;
 uint32_t       type;
} __attribute__ ((packed)) e820range_t;

typedef struct _dub {
 acpi_header_t*  entry;
 uint32_t        rsdt_ptr;
 uint32_t        xsdt_ptr;
} dub_t;

static char const *e820_types[] = {
 "usable",
 "reserved",            // 2
 "ACPI reclaim",        // 3
 "ACPI NVS",            // 4
 "unusable",
};

#define SLOTS_MAXMEMSIZE (sizeof(GRUB4DOS_DRIVE_MAP_SLOT) * MAXIMUM_GRUB4DOS_DRIVE_MAP_SLOTS)
static const char hex[] = "0123456789abcdef";
static const char acpi_rsd_ptr[]="RSD PTR ";

static char		lines_buf[256][256];
static char		ignore_list[256][1000];
static char		find[256];
static char		replace[256];
static tree_t		*ROOT_TREE;
static uint32_t		lines_in_hexfile;
static uint32_t		glb_ignores=0;
static char		*cur_file;
static uint32_t		size_src;
static uint32_t		rsdt_entries;
static uint32_t		xsdt_entries;
static int		acpi_revision;
static acpi_rsdp_v20_t	*glb_rsdp;
static uint32_t		glb_raw_rsdp;
static acpi_rsdt_t	*glb_rsdt;
static uint32_t		glb_raw_rsdt;
static acpi_xsdt_t	*glb_xsdt = NULL;
static uint32_t		glb_raw_xsdt;
static acpi_fadt_t	*glb_facp0 = NULL;
static acpi_fadt_t	*glb_facp1 = NULL;
static uint32_t		glb_raw_facp0;
static uint32_t		glb_raw_facp1;
static uint32_t		raw_base;
static uint32_t		raw_size;
static uint32_t		raw_rsdt_base;
static uint32_t		raw_xsdt_base;
static uint32_t		raw_facp0_base;
static uint32_t		raw_facp1_base;
static e820range_t	e820_ranges[MAXRANGES];
static int		e820_nranges;
static int		map_ds;
static int		map_rsv;
static int		sel_rsdt;
static int		sel_xsdt;
static int		sel_facp0;
static int		sel_facp1;
static __dpmi_meminfo	mapmem;
static __dpmi_meminfo	mapmem_rsv;
static __dpmi_meminfo	tmp_mapmem;

static void		*glb_ebda_end;
static void		*glb_ebda;
static void		*ebda_copy;
static void		*LMB_copy;
static uint32_t		acpimem_copy;
static uint32_t		g4d_addr;
static uint32_t		p_lastfree  = 0;
static bool		rsdt_moved  = 0;
static bool		xsdt_moved  = 0;
static bool		facp0_moved = 0;
static bool		facp1_moved = 0;
static int		dub_max;
uint32_t		shift_pressed;
FILE 			*file_log;

void err_exit(int code) __attribute__ ((noreturn)); // forward def
	
void make_grubmenu() {
 char	buf[16];
 char	buf2[256];
 FILE	*f;
 int	err;

 f = fopen("root.txt", "r");
 if (!f)  {printf("ACPI Patcher Error: Failed opening %s \n", "boot.txt"); err_exit(2);}
 memset(buf, 0, sizeof(buf));
 err=fread(buf, 1, sizeof(buf), f);
 if (err==0)  {printf("ACPI Patcher Error: Failed reading %s \n", "boot.txt"); err_exit(2);}
 fclose(f);

/*
	debug 0
	errorcheck off
	timeout 0
	
	chainloader hd(0,0)/ACPI_PAT/xpldr >nul
	rootnoverify hd(0,0) >nul
	boot >nul
*/
 memset(buf2, 0, sizeof(buf2));
 strcpy(buf2, "debug 0 \n");
 strcat(buf2, "errorcheck off \n");
 strcat(buf2, "timeout 0 \n");
 strcat(buf2, "\n");
 strcat(buf2, "chainloader ");
 strcat(buf2, buf);
 strcat(buf2, "/ACPI_PAT/xpldr >nul \n");
 strcat(buf2, "rootnoverify ");
 strcat(buf2, buf);
 strcat(buf2, " >nul \n");
 strcat(buf2, "boot >nul \n");
 	LOG ("menu.lst: \n%s\n", buf2);

 f = fopen("menu.lst", "w");
 if (!f) {printf("ACPI Patcher Error: Failed creating %s \n", "menu.lst"); err_exit(2);}
 err=fwrite(buf2, 1, strlen(buf2), f);
 if (err==0)  {printf("ACPI Patcher Error: Failed writing %s \n", "menu.lst"); err_exit(2);}
 fclose(f);
}

void err_exit(int code) {
 char ch;
 
 printf("\n");
 printf("Run grub.exe to continue loading OS\n");
 printf("Run ntfs4dos.exe to access NTFS disks for storing logs\n");
 printf("Logs are: main_log.txt tbl_res*.bin err*.txt log*.txt \n");
 printf("\n");
 printf("Press Enter to run Shell\n");
 while (fread(&ch, 1, 1, stdin) == 0) ;

 if (code !=2)
	make_grubmenu(); // avoid recursion
 fclose(file_log);
 exit(1); // debug exit status=1
}

void split_to_lines (FILE *f) {
 uint32_t ch;
 uint32_t cur_in_line=0;

 lines_in_hexfile=0;
 memset(lines_buf, 0, sizeof(lines_buf));
 fseek(f, 0, SEEK_SET);
 ch=fgetc(f);
 while (ch != EOF) {
  switch (ch) {
   case '\x0A':
	if (cur_in_line)
		lines_in_hexfile++;
	cur_in_line=0;
	ch=fgetc(f);
	break;
   case '\x20':
   case '\x0d':
	ch=fgetc(f);
	break;
   case ';':
   case '#':
	ch=fgetc(f);
	while (ch != EOF && ch != '\x0A')
		ch=fgetc(f); // skip all to eol
	break;
   default:
	lines_buf[lines_in_hexfile][cur_in_line]=ch;
	cur_in_line++;
	ch=fgetc(f);
  }
 }

 if (cur_in_line > 0) 
	lines_in_hexfile++; // if EOL missing on last line
}

char* get_line (unsigned int curline) {
 return lines_buf[curline];
}

void split (char *s1) {
 char	*where;

 where =strchr(s1, '/');
 if (where == NULL) {printf("ACPI Patcher Error: '/' not found in hex string %s in file %s\n", s1, cur_file); err_exit(1);}
 unsigned int len 	  = strlen (s1);
 unsigned int len_find	  = where-s1;
 unsigned int len_replace = len-len_find-1;
 
 memset(find, 0, sizeof(find));
 memset(replace, 0, sizeof(replace));
 strncpy(find, s1, len_find);
 strcpy(replace, s1+len_find+1);
 if (len_find % 2 == 1) {printf("ACPI Patcher Error: Error in hex string %s in file %s\n", find, cur_file); err_exit(1);}
 if (len_replace % 2 == 1) {printf("ACPI Patcher Error: Error in hex string %s in file %s\n", replace, cur_file); err_exit(1);}
}

char* split2 (char const *s1) {
 char	*where;

 where =strchr(s1, ','); // "iotr_fix.dif,201904"
 if (where == NULL) 
	 return NULL;
 return where+1; // "201904"
}

unsigned hex2decimal (char ch) {
 if (ch >= '0' && ch <= '9') {
	return (ch - '0');
 } else if (ch >= 'a' && ch <= 'f') {
	return 10 + (ch - 'a');
 } else if (ch >= 'A' && ch <= 'F') {
	return 10 + (ch - 'A');
 }

 return -1;
}

unsigned int char_to_hexnum (char c) {
 if (c >= '0' && c <= '9') 
	return (unsigned int)(c - '0');
 
 if (c >= 'a' && c <= 'f') 
	return (unsigned int)(10 + c - 'a');
 
 if (c >= 'A' && c <= 'F') 
	return (unsigned int)(10 + c - 'A');
 
 return(0);
}

void hexdump (const char *buf, const char *prefix, size_t size) {
 unsigned int b, len, i, c;

 for (b = 0; b < size; b += 16) {
	len = size - b;
	if (len > 16) {
			len = 16;
	}
	
	for (i = 0; i < 16; i++) {
		if (i < len) {
			LOG(" %02x", (unsigned char)buf[b + i]);
		} else {
			LOG("   ");
		}
	}

	LOG(" ");
	for (i = 0; i < len; i++) {
		c = buf[b + i];
		if (c < ' ' || c > '~') {
			c = '.';
		}
		LOG("%c", c);
	}
	
	LOG("\n");
 }
}

void hexdump_raw (const int sel, const char *buf, const char *prefix, size_t size) {
 unsigned int b, len, i, c;

 for (b = 0; b < size; b += 16) {
	len = size - b;
	if (len > 16) {
		len = 16;
	}
	
	for (i = 0; i < 16; i++) {
		if (i < len) {
			LOG(" %02x", (unsigned char) _farpeekb((unsigned short) sel, (uint32_t)buf + b + i ));
		} else {
			LOG("   ");
		}
	}

	LOG(" ");
	for (i = 0; i < len; i++) {
		c = _farpeekb((unsigned short) sel, (uint32_t)buf + b + i );
		if (c < ' ' || c > '~') {
			c = '.';
		}
		LOG("%c", c);
	}
	
	LOG("\n");
 }
}

char* memstr (char *src, unsigned int len_src, char *what, unsigned int len_find) {
 int i;

 for (i=0; i< len_src; i++) {
	if (memcmp(src+i, what, len_find) == 0) {
		return (src+i);
	}
 }
 
 return (NULL);
}

int do_hexreplace (char **p_src, char **p_dst) {
 int	low,high,byte,i;
 int	j;
 char	*where;
 char	*src;
 char	*dst;
 char	buf_find[1024];
 char	buf_replace[1024];
 int	find_len=    strlen(find);
 int	replace_len= strlen(replace);

	//LOG("src size= %d \n",find_len);
	//LOG("find= %s \n",find);
	//LOG("replace size= %d \n",replace_len);
	//LOG("replace= %s \n",replace);
 src=*p_src;
 dst=*p_dst;
 memset(buf_find, 0, sizeof(buf_find));
 memset(buf_replace, 0, sizeof(buf_replace));
 j=0;
 for (i=0;i<find_len;i=i+2) {
	high=hex2decimal(find[i]);
	low=hex2decimal(find[i+1]);
	byte= high*16+low;
	buf_find[j]=byte;
	j++;
 }
 find_len = j;

 j=0;
 for (i=0;i<replace_len;i=i+2) {
	high=hex2decimal(replace[i]);
	low=hex2decimal(replace[i+1]);
	byte= high*16+low;
	buf_replace[j]=byte;
	j++;
 }
 replace_len= j;
 int diff = replace_len-find_len;

 where = memstr(src, size_src, buf_find, find_len);
 if (!where)
	 return(0);

 int hole=where-src;
 memcpy(dst,src,hole);
 memcpy(dst+hole,buf_replace,replace_len);
 memcpy(dst+hole+replace_len,src+hole+find_len,size_src-find_len);
	//LOG("hole = %d\n",hole);
	//LOG("size_src-find_len = %d\n",size_src-find_len);
 size_src += diff;
 memset(src, 0, BUFFERSIZE);
 *p_dst=src;
 *p_src=dst;

 return(1);
}

void e820_insert(uint64_t start, uint64_t len, uint32_t type) {
 if (len == 0) return;
 e820_nranges++;
 e820_ranges[e820_nranges].start=start;
 e820_ranges[e820_nranges].end=start+len;
 e820_ranges[e820_nranges].len=len;
 e820_ranges[e820_nranges].type=type;
}

void e820_init(void) {
 __dpmi_regs	reg;
 e820data_t	ed;
 uint32_t	type;

 memset(&reg, 0, sizeof reg);
 memset(&ed,  0, sizeof ed);
 memset(e820_ranges, 0, sizeof(e820_ranges));
 e820_nranges = 0;
 dosmemput(&ed, sizeof ed, __tb);

 do {
	reg.d.eax = 0x0000e820;
	reg.d.edx = 0x534d4150; // SMAP
	reg.d.ecx = sizeof(e820data_t );
	reg.x.es = __tb >> 4;
	reg.x.di = __tb & 0x0f;
	__dpmi_int (0x15, &reg);
	dosmemget (__tb, sizeof ed, &ed);
	if (reg.x.flags & 1 ||
		reg.d.eax != 0x534d4150 ||
		reg.d.ecx < 20) { break; }

		LOG("%02d %016llx-%016llx (%016llx) t=%d",
		reg.d.ebx, ed.base, ed.base + ed.len, ed.len, ed.type);

		type = ed.type - 1;
		if (type < sizeof(e820_types) / sizeof(e820_types[0]))
			LOG(" %s", e820_types[type]);
		LOG("\n");

	e820_insert(ed.base, ed.len, ed.type);

 } while (reg.d.ebx);
}

uint32_t find_reserv(uint32_t addr, uint32_t *size) {
 int i;

 for (i=0;i<e820_nranges;i++) {
	if (addr >= e820_ranges[i].start &&
	    addr <= e820_ranges[i].end &&
	   (e820_ranges[i].type == 2 || e820_ranges[i].type == 3 || e820_ranges[i].type == 4)) {
			LOG("reserved mem begin=%016llx ", e820_ranges[i].start);
			LOG("size=%016llx \n", e820_ranges[i].len);
		*size = e820_ranges[i].len;
		return e820_ranges[i].start;
	}
 }
 
 *size = 0;
 return 0;
}

uint8_t acpi_checksum (uint8_t *table, uint32_t length) {
 uint8_t ret=0;
 while (length--) {
	ret += *table;
	table++;
 }
 return -ret;
}

uint8_t acpi_checksum_raw (int sel, uint32_t offset, uint32_t length) {
 uint8_t ret=0;
 while (length--) {
	ret += _farpeekb(sel, offset);
	offset++;
 }
 return -ret;
}

void* acpi_get_ebda (void) {
 uint32_t  res;

 res = (uint32_t) _farpeekw(_dos_ds, 0x040E);
	LOG("0x040E=%4x \n", res << 4);
 if (res==0)
	res=0x9fc0;

	LOG("ebda start=%x \n", res << 4);
 return (void*)(res << 4);
}

void* acpi_get_ebda_end (void *ebda) {
 unsigned char ebda_len;

 ebda_len = _farpeekb(_dos_ds,(uint32_t)ebda);
	LOG("ebda len=%x \n",ebda_len);
 if(ebda_len == 0) {
	ebda_len = (BIOS_ROM_LOW - (int)ebda) / 1024;
	_farpokeb(_dos_ds, (uint32_t)ebda, ebda_len);
 }

 return (void*)((int)ebda + ebda_len * 1024);
}

acpi_rsdp_v20_t* acpi_find_rsdp (void) {
 acpi_rsdp_v20_t *rsdp;
 uint32_t ebda_size;

 glb_ebda = acpi_get_ebda();
 glb_ebda_end = acpi_get_ebda_end(glb_ebda);
	LOG("glb_ebda_end=%x \n",(uint32_t) glb_ebda_end);
 ebda_size=glb_ebda_end-glb_ebda;
	LOG("ebda_size=%x \n",ebda_size);
 ebda_copy=malloc(ebda_size);
 dosmemget((uint32_t)glb_ebda,ebda_size,ebda_copy);
	HEXLOG(ebda_copy,"",36 /* ebda_size */);
 LMB_copy=malloc(LMB_size);
 dosmemget((uint32_t)LMB_start,LMB_size,LMB_copy);
	LOG("LMB_copy = %8p \n",LMB_copy);
	HEXLOG(LMB_copy,"", 36 /* LMB_size */);

 // search EBDA
 for ( rsdp = (acpi_rsdp_v20_t *)(ebda_copy);
       rsdp < (acpi_rsdp_v20_t *)((int)ebda_copy + ebda_size - 0x16);
       rsdp = (acpi_rsdp_v20_t *)((unsigned char *)rsdp + 16) ) {
		if ( memcmp(rsdp->rsdpv1.signature, acpi_rsd_ptr, 8) == 0)
		{
			if(rsdp->rsdpv1.revision < 2)
			{
				if(acpi_checksum((uint8_t *)rsdp, RSDP_V1_LENGTH) == 0 )
				{
					glb_raw_rsdp=(uint32_t)rsdp - (uint32_t)ebda_copy + (uint32_t)glb_ebda;
						LOG("EBDA RSD PTR=%08x \n", glb_raw_rsdp);
					return rsdp;
				}
			}
			else
			{
				if(acpi_checksum((uint8_t *)rsdp, ((acpi_rsdp_v20_t*)rsdp)->length) == 0 )
				{
					glb_raw_rsdp=(uint32_t)rsdp - (uint32_t)ebda_copy + (uint32_t)glb_ebda;
						LOG("EBDA RSD XPTR=%08x \n", glb_raw_rsdp);
					return rsdp;
				}
			}
		}
     }
 // search LMB
 for ( rsdp = (acpi_rsdp_v20_t *)(LMB_copy);
       rsdp < (acpi_rsdp_v20_t *)(LMB_copy + LMB_size - 0x16);
       rsdp = (acpi_rsdp_v20_t *)((unsigned char *)rsdp + 16) ) {
		if ( memcmp(rsdp->rsdpv1.signature, acpi_rsd_ptr, 8) == 0)
		{
			if(rsdp->rsdpv1.revision < 2)
			{
				if(acpi_checksum((uint8_t *)rsdp, RSDP_V1_LENGTH) == 0 )
				{
					glb_raw_rsdp=(uint32_t)rsdp - (uint32_t)LMB_copy + LMB_start;
						LOG("LMB RSD PTR=%08x \n", glb_raw_rsdp);
					return rsdp;
				}
			}
			else
			{
				if(acpi_checksum((uint8_t *)rsdp, ((acpi_rsdp_v20_t*)rsdp)->length) == 0 )
				{
					glb_raw_rsdp=(uint32_t)rsdp - (uint32_t)LMB_copy + LMB_start;
						LOG("LMB RSD XPTR=%08x \n", glb_raw_rsdp);
					return rsdp;
				}
			}
		}
 }
 
 return (NULL);
}

uint32_t mapaddr (uint32_t addr) {
 return addr - raw_base + acpimem_copy;
 }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
void acpi_init () {

_farsetsel(_dos_ds);
 glb_rsdp = acpi_find_rsdp();

 if (glb_rsdp == NULL) {printf("ACPI Patcher Error: RSDP table not found in DOS memory\n"); err_exit(1);}
	LOG("glb_rsdp maped=%p \n",glb_rsdp);
	HEXLOG((uint8_t *)glb_rsdp, "", sizeof(acpi_rsdp_v20_t));
	LOG("glb_rsdp raw=%08x \n",glb_raw_rsdp);
	HEXLOG_RAW(_dos_ds, (uint8_t *)glb_raw_rsdp, "", sizeof(acpi_rsdp_v20_t));
 glb_raw_rsdt = (uint32_t) glb_rsdp->rsdpv1.rsdt_address;
 if (glb_raw_rsdt == 0) {printf("ACPI Patcher Error: RSDT address is null\n"); err_exit(1);}

 raw_base=find_reserv(glb_raw_rsdt,&raw_size);
 if (raw_base == 0 || raw_base < BIOS_ROM_HIGH) {printf("ACPI Patcher Error: raw_rsdt=%8x is too low\n", raw_base); err_exit(1);}
	LOG("acpimem raw %08x \n", raw_base);
 raw_size = (raw_size+4095)&(-4096ULL); // 4K align

 mapmem.address=raw_base;
 mapmem.size=raw_size; // acpi res memory
 __dpmi_physical_address_mapping(&mapmem);
 map_ds = __dpmi_allocate_ldt_descriptors(1);
 __dpmi_set_segment_base_address(map_ds, mapmem.address);
 __dpmi_set_segment_limit(map_ds, mapmem.size-1);
 if (mapmem.size != raw_size)  {printf("ACPI Patcher Error: mapmem size mismatch \n"); err_exit(1);}
 acpimem_copy=(uint32_t) malloc(mapmem.size);
 movedata(map_ds, 0, _my_ds(), acpimem_copy, mapmem.size);

 sel_rsdt=map_ds;
 raw_rsdt_base = raw_base;
 sel_xsdt=map_ds;
 raw_xsdt_base = raw_base;
 sel_facp0=map_ds;
 raw_facp0_base = raw_base;
 sel_facp1=map_ds;
 raw_facp1_base = raw_base;

 glb_rsdt=(acpi_rsdt_t *) mapaddr(glb_raw_rsdt);
 if(glb_rsdt == NULL) {printf("ACPI Patcher Error: RSDT not mapped\n"); err_exit(1);}
	LOG("glb_raw_rsdt=%08x : \n",glb_raw_rsdt);
	HEXLOG_RAW(map_ds, (char *) glb_raw_rsdt-raw_base, "", glb_rsdt->header.length);
	LOG("acpimem_copy=%x \n", acpimem_copy);
 rsdt_entries = (glb_rsdt->header.length - sizeof(acpi_header_t))/4;
 xsdt_entries  = 0;
 acpi_revision = 1;
 glb_raw_xsdt  = 0;
	LOG("rsdt_mem  maped:\n");
	uint32_t rsdt_size=glb_rsdt->header.length;
	HEXLOG((char *)glb_rsdt, "", rsdt_size);



 if(glb_rsdp->rsdpv1.revision > 1 && glb_rsdp->length < 1024)
	if(acpi_checksum((uint8_t *)glb_rsdp, glb_rsdp->length) == 0) {
		glb_raw_xsdt = (uint32_t) glb_rsdp->xsdt_xaddr;
		if (glb_raw_xsdt < raw_base ||
		    glb_raw_xsdt > (raw_base+raw_size))
			{printf("ACPI Patcher Error: xsdt addr %08x is in diff reserved memory \n, glb_raw_xsdt"); err_exit(1);}
			
			LOG("glb_raw_xsdt=%08x \n", glb_raw_xsdt);
		glb_xsdt=(acpi_xsdt_t *) mapaddr(glb_raw_xsdt);
			LOG("glb_xsdt=%p \n", glb_xsdt);
			LOG("xsdt_mem maped: \n");
			HEXLOG((char *)glb_xsdt,"",glb_xsdt->header.length);
		xsdt_entries = (glb_xsdt->header.length - sizeof(acpi_header_t))/8;
		acpi_revision = 2;
	}
}

acpi_header_t* find_in_acpi (const char *p_tablename, uint32_t *rsdt_ptr, uint32_t *xsdt_ptr, bool multi) {
 int		i;
 uint32_t	raw_dsdt;
 uint32_t	raw_facs;
 acpi_header_t	*entry;
 acpi_header_t	*ret =NULL;
 int		dub_i, indx=-1;
 dub_t		dub_rsdt[99];
 dub_t		dub_xsdt[99];
 uint8_t	indx_ch[3];
 uint32_t	raw_entry;

 memset(indx_ch, 0, sizeof(indx_ch));
 if (strlen(p_tablename) > 4) {
	strcpy(indx_ch, &p_tablename[4]); // "1",...,"99"
	indx= atoi(indx_ch);
	if (indx == 0) {printf("ACPI Patcher Error: name %s is unknow\n",p_tablename); err_exit(1);}
		//LOG("indx %d %s \n", indx, p_tablename);
 }

 dub_i=0;
 for(i = 0; i < rsdt_entries && i < 100; i++) {
	raw_entry=(uint32_t) glb_rsdt->entry[i];
	entry=(acpi_header_t *) mapaddr(raw_entry);
	if (raw_entry < raw_base ||
	    raw_entry > (raw_base+raw_size))
		{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", raw_entry); err_exit(1);}
		
		//LOG("entry=%08x \n",  raw_entry);
		//HEXLOG((char *)entry, "", 32);
	if(memcmp(entry->signature, "FACP", 4) == 0) {
			//LOG("found facp in rsdt \n");
		if (glb_facp0 == NULL) {
			glb_facp0 = (acpi_fadt_t *) entry;
			glb_raw_facp0 = raw_entry;
			if (glb_raw_facp0 < raw_base ||
			   glb_raw_facp0 > (raw_base+raw_size))
				{printf("ACPI Patcher Error: FACP addr %08x is in diff reserved memory \n", glb_raw_facp0); err_exit(1);}
				
				LOG("glb_raw_facp0=%08x \n", raw_entry);
				LOG("raw_dsdt %08x \n", (uint32_t) glb_facp0->dsdt_addr);
				LOG("raw_facs %08x \n", (uint32_t) glb_facp0->facs_addr);
				HEXLOG_RAW(map_ds, (char *) glb_raw_facp0-raw_base,"", 0x84);
		}

		if(memcmp(p_tablename, "DSDT", 4) == 0) {
			raw_dsdt = (uint32_t) glb_facp0->dsdt_addr;
			if (raw_dsdt < raw_base ||
			    raw_dsdt > (raw_base+raw_size))
				{printf("ACPI Patcher Error: DSDT addr %08x is in diff reserved memory \n", raw_dsdt); err_exit(1);}
			*rsdt_ptr = raw_dsdt;
			ret = (acpi_header_t *) mapaddr(raw_dsdt);
			break;}
		if(memcmp(p_tablename, "FACS", 4) == 0) {
			raw_facs = (uint32_t) glb_facp0->facs_addr;
			if (raw_facs < raw_base ||
			    raw_facs > (raw_base+raw_size))
				{printf("ACPI Patcher Error: FACS addr %08x is in diff reserved memory \n", raw_facs); err_exit(1);}
			*rsdt_ptr = raw_facs;
			ret = (acpi_header_t *) mapaddr(raw_facs);
			break;}
	}

	if(memcmp(entry->signature, p_tablename, 4) == 0)
	{
		if (indx != -1) {
			dub_i++;
			dub_rsdt[dub_i].entry=entry;
			dub_rsdt[dub_i].rsdt_ptr=raw_entry;
			if (dub_i > dub_max) dub_max=dub_i;
			continue;
		}
		if (raw_entry < raw_base ||
		    raw_entry > (raw_base+raw_size))
			{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", raw_entry); err_exit(1);}
		*rsdt_ptr = raw_entry;
		ret = entry;
		break;
	}
 }

 if (dub_i) {
	//printf("dub_i0 %d dub_max %d \n", dub_i, dub_max);
	if (multi == false) {
		if (indx > dub_i) {printf("ACPI Patcher Error: Table %s not found in rsdt list \n", p_tablename); err_exit(1);}
		if (dub_rsdt[indx].rsdt_ptr < raw_base ||
		    dub_rsdt[indx].rsdt_ptr > (raw_base+raw_size))
			{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", dub_rsdt[indx].rsdt_ptr); err_exit(1);}
		*rsdt_ptr = dub_rsdt[indx].rsdt_ptr;
		ret = dub_rsdt[indx].entry;
	}
	else {
		if (indx > dub_i) { // not found in rsdt
			*rsdt_ptr = 0;
			ret = NULL;
		}
		else {
			if (dub_rsdt[indx].rsdt_ptr < raw_base ||
			    dub_rsdt[indx].rsdt_ptr > (raw_base+raw_size))
				{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", dub_rsdt[indx].rsdt_ptr); err_exit(1);}
			*rsdt_ptr = dub_rsdt[indx].rsdt_ptr;
			ret = dub_rsdt[indx].entry;
		}
	}
}

 dub_i=0;
 for(i = 0; i < xsdt_entries && i < 100; i++) {
	raw_entry=(uint32_t) glb_xsdt->entry[i];
	entry=(acpi_header_t *) mapaddr(raw_entry);
	if (raw_entry < raw_base ||
	    raw_entry > (raw_base+raw_size))
		{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", raw_entry); err_exit(1);}
		
		//LOG("entryX=%08x \n", raw_entry);
		//HEXLOG((char *)entry, "", 32);
	if(memcmp(entry->signature, "FACP", 4) == 0) {
			//LOG("found facp in xsdt \n");
		if (glb_facp1 == NULL) {
			glb_facp1 = (acpi_fadt_t *) entry;
			glb_raw_facp1 = raw_entry;
			if (glb_raw_facp1 < raw_base ||
			    glb_raw_facp1 > (raw_base+raw_size))
				{printf("ACPI Patcher Error: FACP addr %08x is in diff reserved memory \n", glb_raw_facp1); err_exit(1);}
				
				LOG("glb_raw_facp1=%08x \n", raw_entry);
				LOG("raw_dsdt1 %08x \n", (uint32_t) glb_facp1->dsdt_xaddr);
				LOG("raw_facs1 %08x \n", (uint32_t) glb_facp1->facs_xaddr);
				HEXLOG_RAW(map_ds, (char *) glb_raw_facp1-raw_base,"", 0x84);
		}

		if(memcmp(p_tablename, "DSDT", 4) == 0) {
			if ((uint32_t) glb_facp1->dsdt_xaddr !=0)
				raw_dsdt = (uint32_t) glb_facp1->dsdt_xaddr;
			else
				raw_dsdt = (uint32_t) glb_facp1->dsdt_addr;
			if (raw_dsdt < raw_base ||
			   raw_dsdt > (raw_base+raw_size))
				{printf("ACPI Patcher Error: DSDT addr %08x is in diff reserved memory \n", raw_dsdt); err_exit(1);}
			*xsdt_ptr=raw_dsdt;
			if (ret != (acpi_header_t*) mapaddr(raw_dsdt))
				{printf("DSDT is diff in rsdt and xsdt, ignoring rsdt \n");}
			return (acpi_header_t *) mapaddr(raw_dsdt); }

		if(memcmp(p_tablename, "FACS", 4) == 0) {
			if ((uint32_t) glb_facp1->facs_xaddr !=0)
				raw_facs = (uint32_t) glb_facp1->facs_xaddr;
			else
				raw_facs = (uint32_t) glb_facp1->facs_addr;
			if (raw_facs < raw_base ||
		            raw_facs > (raw_base+raw_size))
				{printf("ACPI Patcher Error: FACS addr %08x is in diff reserved memory \n", raw_facs); err_exit(1);}
			*xsdt_ptr=raw_facs;
			if (ret != (acpi_header_t*) mapaddr(raw_facs))
				{printf("FACS is diff in rsdt and xsdt, ignoring rsdt \n");}
			return (acpi_header_t *) mapaddr(raw_facs); }
	}

	if(memcmp(entry->signature, p_tablename, 4) == 0)
	{
		if (indx != -1) {
			dub_i++;
			dub_xsdt[dub_i].entry=entry;
			dub_xsdt[dub_i].xsdt_ptr=raw_entry;
			if (dub_i > dub_max) dub_max=dub_i;
			continue;
		}
		if (raw_entry < raw_base ||
		    raw_entry > (raw_base+raw_size))
			{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", raw_entry); err_exit(1);}
		*xsdt_ptr = raw_entry;
		if (ret != entry) {printf("%s is diff in rsdt and xsdt, ignoring rsdt \n", p_tablename);}
		return entry;
	}
 }

 if (dub_i) {
	//printf("dub_i1 %d \n", dub_i);
	if (multi == false) {
		if (indx > dub_i) {printf("ACPI Patcher Error: Table %s not found in xsdt list \n", p_tablename); err_exit(1); }
		if (dub_xsdt[indx].xsdt_ptr < raw_base ||
		    dub_xsdt[indx].xsdt_ptr > (raw_base+raw_size))
			{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", dub_xsdt[indx].xsdt_ptr); err_exit(1);}
		*xsdt_ptr = dub_xsdt[indx].xsdt_ptr;
		if (ret != dub_xsdt[indx].entry) {printf("%s on diff places in rsdt and xsdt, ignoring rsdt \n", p_tablename);}
		return dub_xsdt[indx].entry;
	}
	else {
		if (indx > dub_i) { // not found in xsdt
			*xsdt_ptr = 0;
			return (ret);
		}
		else {
			if (dub_xsdt[indx].xsdt_ptr < raw_base ||
			    dub_xsdt[indx].xsdt_ptr > (raw_base+raw_size))
				{printf("ACPI Patcher Error: addr %08x is in diff reserved memory \n", dub_xsdt[indx].xsdt_ptr); err_exit(1);}
			*xsdt_ptr = dub_xsdt[indx].xsdt_ptr;
			if (ret != dub_xsdt[indx].entry) {printf("%s on diff places in rsdt and xsdt, ignoring rsdt \n", p_tablename);}
			return dub_xsdt[indx].entry;
		}
	}
 }

 return (ret);
}
#pragma GCC diagnostic pop

tree_t* find_node (const char *p_tablename, bool multi) {
 tree_t		*node, *newnode, *prevnode;
 acpi_header_t	*p_acpi, *p_tmp;
 uint32_t	rsdt_ptr, tmp1;
 uint32_t	xsdt_ptr, tmp2;
 FILE		*outfile;

 if (multi == true && dub_max == 1) {
	p_tmp=find_in_acpi(p_tablename, &tmp1, &tmp2, multi); // dummy run to fill dub_max
 }

 node=ROOT_TREE;
 while (node) {
	prevnode=node;
  	if (!strcmp(node->name,p_tablename)) return(node);
	node=node->next;
 }

 if (p_acpi=find_in_acpi(p_tablename, &rsdt_ptr, &xsdt_ptr, multi)) {
		//LOG("new node=%08x for table %s \n", p_acpi, p_tablename);
		//hexdump((uint8_t *)p_acpi,"", 32);
		//outfile=fopen(p_tablename,"wb");
		//fwrite(p_acpi, 1, p_acpi->length, outfile);
		//fclose(outfile);
	newnode=calloc(sizeof(tree_t), 1);
	if (!newnode) {printf("ACPI Patcher Error: Error allocating %d bytes \n", sizeof(tree_t)); err_exit(1);}
		//LOG("alloc newnode = %8x, prevnode= %8x \n", newnode, prevnode);
	strcpy(newnode->name,p_tablename);
	newnode->size=p_acpi->length;
	newnode->next=0;
	newnode->changed=0;
	newnode->ptr=p_acpi;
	newnode->allocated=0;
	newnode->rsdt_ptr=rsdt_ptr;
	newnode->xsdt_ptr=xsdt_ptr;
	newnode->newstore=0;
	prevnode->next=newnode;
	return(newnode);
 }

 return(NULL);
}

void upd_node (tree_t *p_table, char *p_newmem, int newsize) {
 if (p_table->allocated == 0) {
	void *tmpmem=malloc(p_table->size);
		//LOG("alloc tmpmem = %8x \n",tmpmem);
	p_table->ptr=tmpmem;
 }
	//void* old=p_table->ptr;
 void *tmp =realloc(p_table->ptr,newsize);
 if (!tmp) {printf("ACPI Patcher Error: Error reallocating %d bytes \n", newsize); err_exit(1);}
 p_table->ptr=tmp;
	//LOG("realloc old= %8x new= %8x oldsize=%d  new size=%d \n", old, p_table->ptr, p_table->size, newsize);
 memcpy(p_table->ptr,p_newmem,newsize);
 p_table->changed=1;
 p_table->allocated=1;
 p_table->size=newsize;
}

bool rawmem_in_bios_rom(uint32_t target) {
 return (target >= BIOS_ROM_LOW && target <= BIOS_ROM_HIGH);
}

bool rawmem_is_readonly(int sel, uint32_t target, int len) {
 uint8_t test;
 int i;

 for(i = 0; i < len; i++)
 {
	test = _farpeekb(sel, target+i);
	_farpokeb(sel, target+i, ~test);
	if(_farpeekb(sel, target+i) == test)
		return 1;
	_farpokeb(sel, target+i, test);
 }
 
 return 0;
}

void* is_zero_filled(uint32_t target, int len) {
 int i;

 for(i = 0; i < len; i++)
 {
	if(_farpeekb(_dos_ds, target+i) != 0x0)
		return (void*)(target + i); /* return the address where we failed */
 }
 
 return 0;
}

void* find_space(void* start, void* end, int len) {
 void* aligned_target;
 void* non_zero;

 aligned_target = start;
 aligned_target = (void*)ALIGN(aligned_target, 16);
 while(aligned_target >= start && aligned_target < end)
 {
	non_zero = is_zero_filled((uint32_t)aligned_target, len);
	if(non_zero == 0)
		return aligned_target;
	aligned_target = (void*)((int)non_zero + 16); /* start again */
	aligned_target = (void*)(ALIGN(aligned_target, 16));
 }

 return 0;
}

acpi_rsdp_v20_t* alloc_mem_ebda (int size) {
 void * target;
 int alloc_length;

 alloc_length = size + ACPI_PAD;
 target = find_space(glb_ebda + 16, glb_ebda_end - 16, alloc_length);
 target = (void*)(((int)target >> 4) << 4);
 if(target == NULL)
	return NULL;
 else
	return target + ACPI_PAD;
}

void upd_rsdp () {
 uint32_t		alloc_length=RSDP_V1_LENGTH;
 acpi_rsdp_v20_t	*new_rsdp;
 uint8_t		chksum;

	LOG("Update RSDP \n");
 if ( glb_raw_rsdp <= (uint32_t)glb_ebda ||
      glb_raw_rsdp >= (uint32_t)glb_ebda_end ) { // out of ebda
		LOG("RSDP is out of ebda, moving to LOW \n");
	if (acpi_revision < 2)
		alloc_length = RSDP_V1_LENGTH;
	else
		alloc_length = glb_rsdp->length;

	new_rsdp = alloc_mem_ebda(alloc_length);
	if(new_rsdp == NULL) {printf("ACPI Patcher Error: new_rsdp is null\n"); err_exit(1);}

	movedata(_dos_ds, (uint32_t) glb_raw_rsdp, _dos_ds, (uint32_t) new_rsdp, alloc_length);
		HEXLOG_RAW(_dos_ds, (char *) new_rsdp, "", alloc_length);
 }
 else
	new_rsdp=(acpi_rsdp_v20_t *) glb_raw_rsdp;

 #ifdef WRITE
 _farpokel(_dos_ds, (uint32_t) new_rsdp+0x10, glb_raw_rsdt); // 10 rsdt offset
 if (acpi_revision > 1)
	_farpokel(_dos_ds, (uint32_t) new_rsdp+0x18, glb_raw_xsdt); // 18 xsdt offset

 _farpokeb(_dos_ds, (uint32_t) new_rsdp+0x08, 0); // 08 - checksum offset
 chksum=acpi_checksum_raw(_dos_ds, (uint32_t) new_rsdp, RSDP_V1_LENGTH);
 _farpokeb(_dos_ds, (uint32_t) new_rsdp+0x08,chksum);

 if (acpi_revision > 1)	{
	_farpokeb(_dos_ds, (uint32_t) new_rsdp+0x20, 0); // 20 - ext_checksum offset
	chksum=acpi_checksum_raw(_dos_ds, (uint32_t) new_rsdp, alloc_length);
	_farpokeb(_dos_ds, (uint32_t) new_rsdp+0x20,chksum);
 }
 #endif

	LOG("new_rsdp=%08x  raw: \n",(uint32_t) new_rsdp);
	HEXLOG_RAW(_dos_ds, (char *) new_rsdp, "", alloc_length);
	LOG("    chksum=%d \n", acpi_checksum_raw(_dos_ds, (uint32_t)new_rsdp, RSDP_V1_LENGTH));
	if (acpi_revision >= 2)
		LOG("ext_chksum=%d \n", acpi_checksum_raw(_dos_ds, (uint32_t)new_rsdp, alloc_length));
}

void upd_rsdt_xsdt(tree_t *node) {
 int		i;
 acpi_header_t	*entry;
 uint8_t	chksum;

 if (rsdt_moved == 0)
	if (rawmem_in_bios_rom(glb_raw_rsdt) ||
            rawmem_is_readonly(map_ds, glb_raw_rsdt - raw_base, 1)) {
			LOG("Realloc RSDT to writable mem \n");
		movedata(_my_ds(), (uint32_t) glb_rsdt, map_rsv, p_lastfree, glb_rsdt->header.length);
			HEXLOG_RAW(map_rsv, (char *) p_lastfree, "", glb_rsdt->header.length);
		glb_raw_rsdt = p_lastfree + g4d_addr;
			LOG("new glb_raw_rsdt=%08x \n",glb_raw_rsdt);
			LOG("p_lastfree %8x, + %8x \n", p_lastfree+g4d_addr, glb_rsdt->header.length);
		p_lastfree += glb_rsdt->header.length;
		p_lastfree = (p_lastfree+7)&(-8ULL); // align 8 bytes
		sel_rsdt=map_rsv;
		raw_rsdt_base = g4d_addr;
		rsdt_moved = 1;
	}
	
 if (xsdt_moved == 0 && acpi_revision > 1)
	if (rawmem_in_bios_rom(glb_raw_xsdt) ||
	    rawmem_is_readonly(map_ds, glb_raw_xsdt - raw_base, 1)) {
			LOG("Realloc XSDT to writable mem \n");
		movedata(_my_ds(), (uint32_t) glb_xsdt, map_rsv, p_lastfree, glb_xsdt->header.length);
			HEXLOG_RAW(map_rsv, (char *) p_lastfree, "", glb_xsdt->header.length);
		glb_raw_xsdt = p_lastfree + g4d_addr;
			LOG("new glb_raw_xsdt=%08x \n",glb_raw_xsdt);
			LOG("p_lastfree %8x, + %8x \n", p_lastfree+g4d_addr, glb_xsdt->header.length);
		p_lastfree += glb_xsdt->header.length;
		p_lastfree = (p_lastfree+7)&(-8ULL); // align 8 bytes
		sel_xsdt=map_rsv;
		raw_xsdt_base = g4d_addr;
		xsdt_moved = 1;
	}

	for(i = 0; i < rsdt_entries && i < 100; i++) {
		entry=(acpi_header_t *) mapaddr((uint32_t) glb_rsdt->entry[i]);
			//LOG("sig == name %s == %s  entry == rsdt_ptr %08x == %08x \n",
			//     entry->signature, node->name, glb_rsdt->entry[i], node->rsdt_ptr);
		if(memcmp(entry->signature, node->name, 4) == 0 && (uint32_t) glb_rsdt->entry[i] == node->rsdt_ptr) {
				LOG("sig == name %s == %s  entry == rsdt_ptr %08x == %08x \n",
				     entry->signature, node->name, (uint32_t) glb_rsdt->entry[i], node->rsdt_ptr);
				LOG("update pointer [%08x]= %08x :\n",
				     (uint32_t) (glb_raw_rsdt+sizeof(struct acpi_table_header)+i*4), (uint32_t) node->newstore + g4d_addr);
				HEXLOG_RAW(map_rsv, (char *) node->newstore, "", 16);
			#ifdef WRITE
			_farpokel(sel_rsdt, (uint32_t) glb_raw_rsdt+sizeof(struct acpi_table_header)+i*4 - raw_rsdt_base, node->newstore + g4d_addr);
			_farpokeb(sel_rsdt, (uint32_t) glb_raw_rsdt+0x09 - raw_rsdt_base,0); // 09 - checksum offset
			chksum=acpi_checksum_raw(sel_rsdt,  glb_raw_rsdt - raw_rsdt_base, glb_rsdt->header.length);
			_farpokeb(sel_rsdt, (uint32_t) glb_raw_rsdt+0x09 - raw_rsdt_base,chksum);
			#endif
				LOG("new RSDT raw: \n");
				HEXLOG_RAW(sel_rsdt, (char *) glb_raw_rsdt - raw_rsdt_base, "", glb_rsdt->header.length);
			break;
		}
	}

	for(i = 0; i < xsdt_entries && i < 100; i++) {
		entry=(acpi_header_t *) mapaddr((uint32_t) glb_xsdt->entry[i]);
			//LOG("sig == name %s == %s  entry == xsdt_ptr %08x == %08x \n",
			//     entry->signature, node->name, (uint32_t) glb_xsdt->entry[i], node->xsdt_ptr);
		if(memcmp(entry->signature, node->name, 4) == 0 && (uint32_t) glb_xsdt->entry[i] == node->xsdt_ptr) {
				LOG("sig == name %s == %s  entry == xsdt_ptr %08x == %08x \n",
				     entry->signature, node->name, (uint32_t) glb_xsdt->entry[i], node->xsdt_ptr);
				LOG("update pointer [%08x]= %08x :\n",
				     (uint32_t) (glb_raw_xsdt+sizeof(struct acpi_table_header)+i*8), (uint32_t) node->newstore + g4d_addr);
				HEXLOG_RAW(map_rsv, (char *) node->newstore, "", 16);
			#ifdef WRITE
			_farpokel(sel_xsdt, (uint32_t) glb_raw_xsdt+sizeof(struct acpi_table_header)+i*8 - raw_xsdt_base, node->newstore + g4d_addr);
			_farpokeb(sel_xsdt, (uint32_t) glb_raw_xsdt+0x09 - raw_xsdt_base,0); // 09 - checksum offset
			chksum=acpi_checksum_raw(sel_xsdt,  glb_raw_xsdt - raw_xsdt_base, glb_xsdt->header.length);
			_farpokeb(sel_xsdt, (uint32_t) glb_raw_xsdt+0x09 - raw_xsdt_base,chksum);
			#endif
				LOG("new XSDT raw: \n");
				HEXLOG_RAW(sel_xsdt, (char *) glb_raw_xsdt - raw_xsdt_base, "", glb_xsdt->header.length);
			break;
		}
	}
}

void upd_facp(tree_t *node) {
 uint8_t	chksum;
 uint8_t	facp1_ver = 0;
 tree_t 	*newnode;

 if (glb_facp0 == NULL) {printf("ACPI Patcher Error: facp0 is zero \n"); err_exit(1);}
 if (glb_facp1) {
	if (glb_facp1->header.length <= 116) facp1_ver =1;
	else facp1_ver =2;
 }

 if (facp0_moved == 0)
	if (rawmem_in_bios_rom(glb_raw_facp0) ||
	    rawmem_is_readonly(map_ds, glb_raw_facp0 - raw_base, 1))  {
		LOG("Realloc facp0 to writable mem \n");
	movedata(_my_ds(), (uint32_t) glb_facp0, map_rsv, p_lastfree, glb_facp0->header.length);
		HEXLOG_RAW(map_rsv, (char *) p_lastfree, "", glb_facp0->header.length);
	uint32_t old_raw_facp0 = glb_raw_facp0;
	glb_raw_facp0 = p_lastfree + g4d_addr;
		LOG("new glb_raw_facp0=%08x \n",glb_raw_facp0);
		LOG("p_lastfree %8x, + %8x \n", p_lastfree+g4d_addr, glb_facp0->header.length);
	sel_facp0=map_rsv;
	raw_facp0_base = g4d_addr;
	facp0_moved = 1;

	newnode=calloc(sizeof(tree_t), 1);
	if (!newnode) {printf("ACPI Patcher Error: Error allocating %d bytes \n", sizeof(tree_t)); err_exit(1);}
	strcpy(newnode->name,"FACP");
	newnode->size=glb_facp0->header.length;
	newnode->changed=1;
	newnode->allocated=0;
	newnode->newstore=p_lastfree;
	newnode->ptr=(void *)-1;
	newnode->rsdt_ptr=old_raw_facp0;
	newnode->xsdt_ptr=0;
	newnode->next=node->next;
	node->next=newnode;
	
	p_lastfree += glb_facp0->header.length;
	p_lastfree = (p_lastfree+7)&(-8ULL); // align 8 bytes
 }

 if (facp1_ver && facp1_moved == 0)
	if (rawmem_in_bios_rom(glb_raw_facp1) ||
            rawmem_is_readonly(map_ds, glb_raw_facp1 - raw_base, 1))  {
		LOG("Realloc facp1 to writable mem \n");
	movedata(_my_ds(), (uint32_t) glb_facp1, map_rsv, p_lastfree, glb_facp1->header.length);
		HEXLOG_RAW(map_rsv, (char *) p_lastfree, "", glb_facp1->header.length);
	uint32_t old_raw_facp1 = glb_raw_facp1;
	glb_raw_facp1 = p_lastfree + g4d_addr;
		LOG("new glb_raw_facp1=%08x \n",glb_raw_facp1);
		LOG("p_lastfree %8x, + %8x \n", p_lastfree+g4d_addr, glb_facp1->header.length);
	sel_facp1=map_rsv;
	raw_facp1_base = g4d_addr;
	facp1_moved = 1;
	
	newnode=calloc(sizeof(tree_t), 1);
	if (!newnode) {printf("ACPI Patcher Error: Error allocating %d bytes \n", sizeof(tree_t)); err_exit(1);}
	strcpy(newnode->name,"FACP");
	newnode->size=glb_facp1->header.length;
	newnode->changed=1;
	newnode->allocated=0;
	newnode->newstore=p_lastfree;
	newnode->ptr=(void *)-1;
	newnode->rsdt_ptr=0;
	newnode->xsdt_ptr=old_raw_facp1;
	newnode->next=node->next;
	node->next=newnode;
	
	p_lastfree += glb_facp1->header.length;
	p_lastfree = (p_lastfree+7)&(-8ULL); // align 8 bytes
 }

	LOG("facp0->facs_addr=  %08x facp0->dsdt_addr=  %08x \n\n", (uint32_t) glb_facp0->facs_addr, (uint32_t) glb_facp0->dsdt_addr);
	if (facp1_ver ==2) {
		LOG("facp1->facs_addr=  %08x facp1->dsdt_addr=  %08x \nfacp1->facs_xaddr= %08x facp1->dsdt_xaddr= %08x \n\n",
				(uint32_t) glb_facp1->facs_addr, (uint32_t) glb_facp1->dsdt_addr, (uint32_t) glb_facp1->facs_xaddr, (uint32_t) glb_facp1->dsdt_xaddr); }
	else if (facp1_ver ==1) {
		LOG("facp1->facs_addr=  %08x facp1->dsdt_addr=  %08x \n\n", (uint32_t) glb_facp1->facs_addr, (uint32_t) glb_facp1->dsdt_addr);
	}
	LOG("facp0  raw: \n");
	HEXLOG_RAW(sel_facp0, (char *) glb_raw_facp0-raw_facp0_base, "", sizeof(acpi_header_t)+8);
	LOG("facp1  raw: \n");
	if (facp1_ver) HEXLOG_RAW(sel_facp1, (char *) glb_raw_facp1-raw_facp1_base, "", 0x94);

 if (memcmp(node->name, "DSDT", 4) == 0) {
	#ifdef WRITE
	_farpokel(sel_facp0, (uint32_t) & ((acpi_fadt_t *)glb_raw_facp0)->dsdt_addr - raw_facp0_base, node->newstore + g4d_addr);
	_farpokeb(sel_facp0, (uint32_t) glb_raw_facp0+0x09 - raw_facp0_base,0); // 09 - checksum offset
	chksum=acpi_checksum_raw(sel_facp0, glb_raw_facp0-raw_facp0_base, glb_facp0->header.length);
	_farpokeb(sel_facp0, (uint32_t) glb_raw_facp0+0x09 - raw_facp0_base,chksum);

	if (facp1_ver) {
		_farpokel(sel_facp1, (uint32_t) & ((acpi_fadt_t *)glb_raw_facp1)->dsdt_addr - raw_facp1_base, node->newstore + g4d_addr);
		if (facp1_ver ==2) {
			_farpokel(sel_facp1, (uint32_t) & ((acpi_fadt_t *)glb_raw_facp1)->dsdt_xaddr - raw_facp1_base, node->newstore + g4d_addr);
		}
		_farpokeb(sel_facp1, (uint32_t) glb_raw_facp1+0x09 - raw_facp1_base,0); // 09 - checksum offset
		chksum=acpi_checksum_raw(sel_facp1, glb_raw_facp1-raw_facp1_base, glb_facp1->header.length);
		_farpokeb(sel_facp1, (uint32_t) glb_raw_facp1+0x09 - raw_facp1_base,chksum);
	}
	#endif
 }
 else { // FACS
	#ifdef WRITE
	_farpokel(sel_facp0, (uint32_t) & ((acpi_fadt_t *)glb_raw_facp0)->facs_addr - raw_facp0_base, node->newstore + g4d_addr);
	_farpokeb(sel_facp0, (uint32_t) glb_raw_facp0+0x09 - raw_facp0_base,0); // 09 - checksum offset
	chksum=acpi_checksum_raw(sel_facp0, glb_raw_facp0-raw_facp0_base, glb_facp0->header.length);
	_farpokeb(sel_facp0, (uint32_t) glb_raw_facp0+0x09 - raw_facp0_base,chksum);

	if (facp1_ver) {
		_farpokel(sel_facp1, (uint32_t) & ((acpi_fadt_t *)glb_raw_facp1)->facs_addr - raw_facp1_base, node->newstore + g4d_addr);
		if (facp1_ver ==2) {
			_farpokel(sel_facp1, (uint32_t) & ((acpi_fadt_t *)glb_raw_facp1)->facs_xaddr - raw_facp1_base, node->newstore + g4d_addr);
		}
		_farpokeb(sel_facp1, (uint32_t) glb_raw_facp1+0x09 - raw_facp1_base,0); // 09 - checksum offset
		chksum=acpi_checksum_raw(sel_facp1, glb_raw_facp1-raw_facp1_base, glb_facp1->header.length);
		_farpokeb(sel_facp1, (uint32_t) glb_raw_facp1+0x09 - raw_facp1_base,chksum);
	}
	#endif
 }
 
	LOG("newstore %08x  raw: \n", node->newstore + g4d_addr);
	HEXLOG_RAW(map_rsv, (char *) node->newstore, "", 32);
	LOG("facp0  raw: \n");
	HEXLOG_RAW(sel_facp0, (char *) glb_raw_facp0-raw_facp0_base, "", sizeof(acpi_header_t)+8);
	if (facp1_ver) {
		LOG("facp1  raw: \n");
		HEXLOG_RAW(sel_facp1, (char *) glb_raw_facp1-raw_facp1_base, "", 0x94);
	}
}

int get_tmp_selector(uint32_t addr) {
 int sel;

 tmp_mapmem.address=addr;
 tmp_mapmem.size=32;
 sel = __dpmi_allocate_ldt_descriptors(1);
 __dpmi_physical_address_mapping(&tmp_mapmem);
 __dpmi_set_segment_base_address(sel, tmp_mapmem.address);
 __dpmi_set_segment_limit(sel, 32-1);
 if (tmp_mapmem.size != 32)  {printf("ACPI Patcher Error: tmp_mapmem size mismatch \n"); err_exit(1);}
 
 return sel;
}

int get_resv_selector() {
 GRUB4DOS_DRIVE_MAP_SLOT buf_slots[MAXIMUM_GRUB4DOS_DRIVE_MAP_SLOTS];
 int		i=0;
 bool		found=0;
 int		found_i=0;
 int		fd0=-1;
 int		fd1=-1;
 uint8_t	buf[32];
 uint8_t	buf2[32];
 void		*ptr;
 int		sel=0;
 void		*low_copy;
 uint32_t	addr;
 uint32_t	g4d_size;

 uint32_t	int13vec=	_farpeekl(_dos_ds, 0x13*4);
 uint16_t	int13vec_seg=	(int13vec&0xffff0000) >> 16;
 uint16_t	int13vec_off=	int13vec&0x0000ffff;
 uint32_t	int13addr=	int13vec_seg*16 + int13vec_off;
 uint32_t	slotsaddr=	int13vec_seg*16 + 0x20;

 dosmemget(int13addr, 32, buf); // small buffer
 if (memcmp(buf+3, "$INT13SFGRUB4DOS", 16) != 0) {
	// search 9000-A000 range
	slotsaddr = 0;
	low_copy=malloc(INT13_RANGE_SZ);
	if (!low_copy) {printf("ACPI Patcher Error: Error allocating %d bytes \n", INT13_RANGE_SZ); err_exit(1);}
	dosmemget((uint32_t)INT13_RAM_LOW,INT13_RANGE_SZ,low_copy);
	for (ptr = low_copy;  ptr < low_copy + (INT13_RANGE_SZ - 0x16); ptr++) {
		if (memcmp(ptr, "$INT13SFGRUB4DOS", 16) == 0) {
			slotsaddr=(uint32_t) (ptr-low_copy+INT13_RAM_LOW-0xE3);
				LOG("found int13vec in RAM %08x \n", (uint32_t) (ptr-low_copy+INT13_RAM_LOW));
			break;
		}
	}
	free(low_copy);
	if (slotsaddr == 0) {printf("ACPI Patcher Error: Grub4Dos INT13 handler not found in range 9000-A000\n"); err_exit(1);}
 }
	LOG("int13vec maped %08x : \n", int13addr);
	HEXLOG((char *) buf, "", 64);
 dosmemget(slotsaddr, SLOTS_MAXMEMSIZE, buf_slots); // small buffer
	LOG("slotsaddr maped %08x : \n", slotsaddr);
	HEXLOG((char *) &buf_slots[0], "", sizeof(GRUB4DOS_DRIVE_MAP_SLOT));
	HEXLOG((char *) &buf_slots[1], "", sizeof(GRUB4DOS_DRIVE_MAP_SLOT));
	HEXLOG((char *) &buf_slots[2], "", sizeof(GRUB4DOS_DRIVE_MAP_SLOT));
 for (i=0; i<MAXIMUM_GRUB4DOS_DRIVE_MAP_SLOTS && buf_slots[i].to_drive; i++) {
		LOG("slot %d from=%2d start= %016llx\n", i, buf_slots[i].from_drive, buf_slots[i].start_sector*512);

	if (buf_slots[i].from_drive == 00 && buf_slots[i].to_drive == 0xFF) {
		fd0= i;
		addr=buf_slots[i].start_sector*512;
		sel=get_tmp_selector(addr);
		movedata(sel, 0, _my_ds(), (uint32_t) buf2, 32);
			LOG("fd0 maped %8x : \n", addr);
			HEXLOG(buf2,"", 32);
		if (memcmp(buf2+3, "ACPIPATC", 8) != 0) fd0=-1; // magic string
		if (sel) __dpmi_free_ldt_descriptor(sel);
		__dpmi_free_physical_address_mapping(&tmp_mapmem);
		}

	if (buf_slots[i].from_drive == 01 && buf_slots[i].to_drive == 0xFF) {
		fd1= i;
		addr=buf_slots[i].start_sector*512;
		sel=get_tmp_selector(addr);
		movedata(sel, 0, _my_ds(), (uint32_t) buf2, 32);
			LOG("fd1 maped %8x : \n", addr);
			HEXLOG(buf2,"", 32);
		if (memcmp(buf2+3, "ACPIPATC", 8) != 0) fd1=-1;
		if (sel) __dpmi_free_ldt_descriptor(sel);
		__dpmi_free_physical_address_mapping(&tmp_mapmem);
		}

	if (buf_slots[i].from_drive == 33 && buf_slots[i].to_drive == 0xFF) {
		found=1;
		found_i= i;
		addr=buf_slots[i].start_sector*512;
		sel=get_tmp_selector(addr);
		movedata(sel, 0, _my_ds(), (uint32_t) buf2, 32);
			LOG("ramdisk maped %8x : \n", addr);
			HEXLOG(buf2,"", 32);
		#ifdef WRITE
		if (memcmp(buf2+3, "ACPIPATC", 8) != 0) {found=0; found_i=0;}
		#endif
		if (sel) __dpmi_free_ldt_descriptor(sel);
		__dpmi_free_physical_address_mapping(&tmp_mapmem);
		}
 }
 if (found == 0) {printf("ACPI Patcher Error: ramdisk (type=33) not found\n"); err_exit(1);}

 if (fd0 != -1) {
	uint32_t fd0_addr=slotsaddr + fd0*sizeof(GRUB4DOS_DRIVE_MAP_SLOT);
	#ifdef WRITE
	_farpokew(_dos_ds, fd0_addr, 0);
	#endif
 }
 if (fd1 != -1) {
	uint32_t fd1_addr=slotsaddr + fd1*sizeof(GRUB4DOS_DRIVE_MAP_SLOT);
	#ifdef WRITE
	_farpokew(_dos_ds,fd1_addr, 0);
	#endif
 }
	LOG("slotsaddr %08x  raw: \n", slotsaddr);
	HEXLOG_RAW(_dos_ds, (char *) slotsaddr, "", sizeof(GRUB4DOS_DRIVE_MAP_SLOT));
	HEXLOG_RAW(_dos_ds, (char *) slotsaddr+1*sizeof(GRUB4DOS_DRIVE_MAP_SLOT), "", sizeof(GRUB4DOS_DRIVE_MAP_SLOT));
	HEXLOG_RAW(_dos_ds, (char *) slotsaddr+2*sizeof(GRUB4DOS_DRIVE_MAP_SLOT), "", sizeof(GRUB4DOS_DRIVE_MAP_SLOT));

 g4d_addr= buf_slots[found_i].start_sector*512;
 g4d_size= buf_slots[found_i].sector_count*512;

 mapmem_rsv.address=g4d_addr;
 mapmem_rsv.size=g4d_size;
	LOG("acpi store address= %08x ", mapmem_rsv.address);
	LOG("size= %08x \n", mapmem_rsv.size);
 sel = __dpmi_allocate_ldt_descriptors(1);
 __dpmi_physical_address_mapping(&mapmem_rsv);
 __dpmi_set_segment_base_address(sel, mapmem_rsv.address);
 __dpmi_set_segment_limit(sel, mapmem_rsv.size-1);
 if (mapmem_rsv.size != g4d_size)  {	printf("ACPI Patcher Error: mapmem_rsv size mismatch \n"); err_exit(1);}
 
 return sel;
}

void finish() {
 tree_t		*node;
 tree_t		*lastnode;
 acpi_header_t	*p;
 FILE		*outfile;
 int		i=0;
 char		s[8+5], s1[3];

 map_rsv= get_resv_selector();
 if (map_rsv == 0) {printf("ACPI Patcher Error: Selector for storing tables is null\n"); err_exit(1);}
 node=ROOT_TREE;
 while (node) {
	if (node->ptr && node->changed) {
		if (node->ptr != (void *) -1) {
			p=node->ptr;
			p->length = node->size;
			if (memcmp(node->name, "FACS", 4) != 0) { // skip facs
				p->checksum = 0x0;
				p->checksum = acpi_checksum ((uint8_t *) p, node->size);
			}
			node->newstore =p_lastfree;
			movedata(_my_ds(), (uint32_t) p, map_rsv, p_lastfree, node->size); // ds:p -> map_rsv:p_lastfree
				//LOG("p_lastfree %8x, + %8x \n", p_lastfree+g4d_addr, node->size);
			p_lastfree += node->size;
			p_lastfree = (p_lastfree+7)&(-8ULL); // align 8 bytes
		}
		if ( memcmp(node->name, "DSDT", 4) == 0 || memcmp(node->name, "FACS", 4) == 0) {
				//LOG("table %s dumping \n", node->name);
			upd_facp(node);
		}
		else {
				//LOG("table %s dumping \n", node->name);
			upd_rsdt_xsdt(node);
		}
	}
	lastnode=node;
	node=node->next;
 }

 if (rsdt_moved || xsdt_moved) {
	upd_rsdp();
 }

////////////////////////////////////
// Flush tables to disk for debug //
///////////////////////////////////
 node=ROOT_TREE;
 while (node) {
	if (node->ptr && node->changed && node->ptr != (void *) -1) {
		sprintf(s1,"%d",i);
		strcpy(s, "tbl_res");
		strcat(s, s1);
		strcat(s, ".bin");
		outfile=fopen(s,"wb");
		if (!outfile) {printf("ACPI Patcher Error: Error opening %s \n", s); err_exit(1);}
		fwrite(node->ptr, 1, node->size, outfile);
		fclose(outfile);
		i++;
	}
	lastnode=node;
	node=node->next;
 }
////////////////////////////////

 if (map_ds) 	__dpmi_free_ldt_descriptor(map_ds);
 __dpmi_free_physical_address_mapping(&mapmem);
 if (map_rsv) 	__dpmi_free_ldt_descriptor(map_rsv);
 __dpmi_free_physical_address_mapping(&mapmem_rsv);
 if (sel_rsdt) 	__dpmi_free_ldt_descriptor(sel_rsdt);
 if (sel_xsdt) 	__dpmi_free_ldt_descriptor(sel_xsdt);
 if (sel_facp0) 	__dpmi_free_ldt_descriptor(sel_facp0);
 if (sel_facp1) 	__dpmi_free_ldt_descriptor(sel_facp1);

 if (acpimem_copy)	free((void *)acpimem_copy);
 if (ebda_copy) 		free(ebda_copy);
 if (LMB_copy) 		free(LMB_copy);

 node=ROOT_TREE;
 while (node) {
	if (node->ptr && node->allocated && node->ptr != (void *) -1) {
			//LOG("free node->ptr= %8x \n",node->ptr);
		free(node->ptr);
	}
	lastnode=node;
	node=node->next;
		//LOG("free node= %8x, next= %8x \n",lastnode, node);
	free(lastnode);
 }
}

void hexpatch (const char *p_tablename, const char *filename) {
 FILE		*f;
 uint32_t	curline;
 uint32_t	res=0;
 char		*line;
 char		*mem;
 char		*mem2;
 char		**p_mem;
 char		**p_mem2;
 tree_t		*p_table; // pointer to local copy of table
 bool		multi=0;
 char		s[4+2+1], s1[2+1];
 int		i;

 mem=    calloc(BUFFERSIZE,1);
 mem2=   calloc(BUFFERSIZE,1);
 p_mem=  &mem;
 p_mem2= &mem2;

 f = fopen(filename, "r");
 if (!f) {printf("ACPI Patcher Error: Failed opening %s \n", filename); err_exit(1);}
 if (fseek(f, 0, SEEK_END)) {printf("ACPI Patcher Error: Failed seeking %s \n", filename); err_exit(1);}

 if (strlen(p_tablename) > 4) {
	if (p_tablename[4] == '*') {
		multi=1;
	}
 }

 if (multi==1) {
	dub_max=1;
	for (i=1; i<=dub_max && i<=99; i++) {
		sprintf(s1,"%d",i);
		memset(s,0,sizeof(s));
		memcpy(s, p_tablename,4); 
		strcat(s, s1); // SSDT99
		p_table=find_node(s, NOERR);
		if (p_table) {
			size_src = p_table->size;
			memcpy(mem, p_table->ptr, size_src);

			split_to_lines(f);
			for (curline=0; curline < lines_in_hexfile; curline++ ) {
				line=get_line(curline);
				split(line);
				if (do_hexreplace(p_mem, p_mem2)) res=1;
			}

			if (res) {
					LOG("table %s changed, new size= %u \n", p_tablename, size_src);
				upd_node(p_table, *p_mem, size_src);
			}
		}
	}
 }
 else {
	p_table=find_node(p_tablename, ERR);
	if (p_table) {
		size_src = p_table->size;
		memcpy(mem, p_table->ptr, size_src);

		split_to_lines(f);
		for (curline=0; curline < lines_in_hexfile; curline++ ) {
			line=get_line(curline);
			split(line);
			if (do_hexreplace(p_mem, p_mem2)) res=1;
		}

		if (res) {
				LOG("table %s changed, new size= %u \n", p_tablename, size_src);
			upd_node(p_table, *p_mem, size_src);
		}
	}
	else {printf("ACPI Patcher Error: Table %s not found\n", p_tablename); err_exit(1);}
 }

 fclose(f);
 free(mem);
 free(mem2);
}

void binpatch (const char *p_tablename, const char *filename) {
 FILE		*f;
 int 		loaded, size;
 uint8_t	*mem;
 tree_t		*p_table;

 f = fopen(filename, "rb");
 if (!f) {printf("ACPI Patcher Error: Failed opening %s \n", filename); err_exit(1);}
 if (fseek(f, 0, SEEK_END)) {printf("ACPI Patcher Error: Failed seeking %s \n", filename); err_exit(1);}
 size=ftell(f);
 if (size == 0) {printf("ACPI Patcher Error: Failed opening %s, size is zero \n", filename); err_exit(1);}
 fseek(f, 0, SEEK_SET);
  
 mem= malloc(size);
 if (!mem) {printf("ACPI Patcher Error: Error allocating %d bytes \n", size); err_exit(1);}
 loaded=fread(mem, 1, size, f);
 if (size != loaded)  {printf("ACPI Patcher Error: Failed loading %s, size mitmatch \n", filename); err_exit(1);}
 
 p_table=find_node(p_tablename, ERR);
 if (p_table) {
			LOG("table %s replaced, new size= %d \n", p_tablename, size);
		upd_node(p_table, mem, size);
	}
 else {printf("ACPI Patcher Error: Table %s not found\n", p_tablename); err_exit(1);}
  
 fclose(f);
 free(mem);
 }
 
void remove_crlf (char * string) {
 int	len;
 char	lastchar, lastlastchar; 
 
 len=strlen(string);
 lastchar=string[len-1];
 lastlastchar=string[len-2];
 if (lastchar == '\n' || lastchar == '\r') string[len-1]=0;
 if (lastlastchar == '\n' || lastlastchar == '\r') string[len-2]=0;
}
  
void load_ignorelist () {
 int	i;
 char	*res;	
 FILE	*f;
 
 f=fopen("ignores.txt","r");
 if (!f)
	return;
 i=1;
 while (res=fgets(ignore_list[i], 1000, f)) {
	remove_crlf(ignore_list[i]);
	i++; }
 glb_ignores=i-1;
 fclose(f);
 }
 
void difpatch (const char *p_tablename, const char *filename) {
 FILE		*f;
 FILE		*outfile;
 int		loaded, size;
 uint8_t	*mem;
 tree_t		*p_table;
 tree_t		*p_target;
 char		s[4+2+1], s1[2+1], s3[6+4+1], errbuf[4096], filename2[8+4+1];
 char		list3[100][13];
 char		*list2[100];
 int		i, ret, j=0, errsize, i2, i3;
 char		compiler_name[4+1];
 uint32_t	compiler_ver;
 acpi_header_t	*header;
 char		iasl[8+5];
 char		*force_ver;
 char		*res, *res2;
 bool		found_ignores=0, skip_ssdt=1;

 if (memcmp(p_tablename, "SSDT", 4) != 0 &&
	 memcmp(p_tablename, "DSDT", 4) != 0 ) {printf("ACPI Patcher Error: Diffing %s is not implemented \n", p_tablename); err_exit(1); }
 
 if (glb_ignores == 0 ) load_ignorelist();
 
 p_table=find_node("SSDT1", NOERR); // check for any ssdt presence
 if (p_table) skip_ssdt=0;
 	 
 p_table=find_node(p_tablename, ERR);
 if (p_table) {
	p_target=p_table;
	/* 
	dsdt: iasl*.exe -fe external.txt -e ssdt1.bin ssdt2.bin ssdt*.bin -d target.bin > err1/log1.txt
	ssdt: iasl*.exe -fe external.txt -e ssdt2.bin ssdt3.bin ssdt*.bin dsdt.bin  -d target.bin > err1/log1.txt
	
	patch -u target.dsl %filename% -o target_p.dsl > err2/log2.txt
	
	iasl*.exe -ve target_p.dsl > err3/log3.txt
	
	binpatch(p_tablename, target_p.aml)	
	*/
	outfile=fopen("target.bin","wb");
	if (!outfile) {printf("ACPI Patcher Error: Error opening target.bin \n"); err_exit(1);}
	fwrite(p_table->ptr, 1, p_table->size, outfile);
	fclose(outfile);

	dub_max=1;
	memset(list2,0,sizeof(list2));
	memset(list3,0,sizeof(list3));
	
	header=(acpi_header_t *) p_target->ptr;
	memset(compiler_name,0,sizeof(compiler_name));
	memcpy(&compiler_name, header->asl_compiler_id, 4);
	compiler_ver= header->asl_compiler_revision;
	memset(iasl,0,sizeof(iasl));
	if 	(!memcmp(compiler_name, "INTL", 4) && compiler_ver >  0x20161231 ) //&& compiler_ver <= 0x20191231)
		strcpy(iasl,"ia201904.exe");
	else if (!memcmp(compiler_name, "INTL", 4) && compiler_ver <= 0x20161231)
		strcpy(iasl,"ia201612.exe");
	
	memset(filename2,0,sizeof(filename2));
	force_ver=split2(filename);
	if (force_ver) {
		strcpy(iasl,"ia");
		strcat(iasl,force_ver);
		strcat(iasl,".exe");
		strncpy(filename2, filename, (int)(force_ver-filename-1)); // -1 = ","
	}
	else 	{
		strcpy(filename2,filename);
	}
		//LOG("filename2: %s \n", filename2);
	f = fopen(iasl, "rb");
	if (!f) {printf("ACPI Patcher Error: %s not found \n", iasl); err_exit(1);}
	fclose(f);
	
	strcpy(list3[j], "redir.exe"); 		list2[j]= list3[j]; j++;
	strcpy(list3[j], "-o");			list2[j]= list3[j]; j++;
	strcpy(list3[j], "log1.txt");		list2[j]= list3[j]; j++;
	strcpy(list3[j], "-e");			list2[j]= list3[j]; j++;
	strcpy(list3[j], "err1.txt");		list2[j]= list3[j]; j++;
	
	list2[j]= iasl; j++;
	strcpy(list3[j], "-fe");		list2[j]= list3[j]; j++;
	strcpy(list3[j], "external.txt");	list2[j]= list3[j]; j++;
	if (skip_ssdt==0)
		{strcpy(list3[j], "-e");	list2[j]= list3[j]; j++;}
	
	for (i=1; i<=dub_max && i<=99 && skip_ssdt==0; i++) {
		sprintf(s1,"%d",i);
		memset(s,0,sizeof(s));
		memset(s3,0,sizeof(s3));
		strcpy(s, "SSDT");
		strcat(s, s1); // SSDT99
		strcpy(s3, s); 
		if (strcmp(p_tablename, s3) != 0) {
			strcat(s3, ".bin");
			strcpy(list3[j], s3); list2[j]= list3[j]; j++;
				//printf("enum %s [%s] \n", s, s1);
			p_table=find_node(s, NOERR);
			if (p_table) {
				outfile=fopen(s3,"wb");
				if (!outfile) {printf("ACPI Patcher Error: Error opening %s \n", s3); err_exit(1);}
				fwrite(p_table->ptr, 1, p_table->size, outfile);
				fclose(outfile);
			}
		}
	}

	if (strcmp(p_tablename, "DSDT") != 0) {
		p_table=find_node("DSDT", ERR);
		if (p_table) {
			strcpy(list3[j], "DSDT.bin"); list2[j]= list3[j]; j++;
			outfile=fopen("DSDT.bin","wb");
			if (!outfile) {printf("ACPI Patcher Error: Error opening DSDT.bin \n"); err_exit(1);}
			fwrite(p_table->ptr, 1, p_table->size, outfile);
			fclose(outfile);
		}
	}
	
	strcpy(list3[j], "-d");		list2[j]= list3[j]; j++;
	strcpy(list3[j], "target.bin");	list2[j]= list3[j]; j++;
	ret=spawnv(P_WAIT, "redir.exe", list2);
		i=0; while (list2[i] != NULL) { LOG("%s ", list2[i++]); }; LOG("\n");		
		LOG("errorcode decomp: %d \n", ret);
		
	f = fopen("err1.txt", "r");
	if (!f) {printf("ACPI Patcher Error: Failed opening err1.txt \n"); err_exit(1);}
	if (fseek(f, 0, SEEK_END)) {printf("ACPI Patcher Error: Failed opening err1.txt \n"); err_exit(1);}
	errsize=ftell(f);
	fseek(f, 0, SEEK_SET);
	if (errsize != 0) {
		memset(errbuf, 0, sizeof(errbuf));
			//LOG("glb_ignores: %d \n",glb_ignores);
		if (glb_ignores != 0 ) {
			while (res=fgets(errbuf, sizeof(errbuf), f)) {
					LOG("errbuf: %s \n",errbuf);
				found_ignores=0;
				for (i3=1; i3 <= glb_ignores; i3++) {
					res2=strstr(errbuf, ignore_list[i3]);
					if (res2 !=  NULL ) {
							LOG("found ignore: %s \n",ignore_list[i3]);
						found_ignores=1;
						break;
					}
				}
				
				if (found_ignores==0) {
					printf("ACPI Patcher Error: Failed decompiling %s \n", p_tablename);
					printf("%s \n",errbuf);
					err_exit(1);			
				}
				
				memset(errbuf, 0, sizeof(errbuf));
			}
		}
		else {  // glb_ignores = 0, file ignores.txt doesnt exist
			res=fgets(errbuf, sizeof(errbuf), f);
			printf("ACPI Patcher Error: Failed decompiling %s \n", p_tablename);
			printf("%s \n",errbuf);
			err_exit(1);	
		}
	}
	fclose(f);

	char *patch_args[] = { // max cmdline size 125 !!! limit of freecom !!!
	"redir.exe",
	"-o",
	"log2.txt",
	"-e",
	"err2.txt",
	
	"patch.exe",
	"-t",		//--batch
	"-F",		//--fuzz
	"2",		// default is 2
	"-s",		//--silent
	"-l",		//--ignore-whitespace
	"-u",		//--unified
	"target.dsl",
	"-i",		//--input
	(char *) filename2,
	"-o",		//--output
	"target_p.dsl",
	"-r",		//--reject-file
	"target.rej",
	0 };
	
	ret=spawnv(P_WAIT, "redir.exe", patch_args);
		i=0; while (patch_args[i] != NULL) { LOG("%s ", patch_args[i++]); }; LOG("\n");
		LOG("errorcode patch: %d \n", ret);
		
	f = fopen("err2.txt", "r");
	if (!f) {printf("ACPI Patcher Error: Failed opening err2.txt \n"); err_exit(1);}
	if (fseek(f, 0, SEEK_END)) {printf("ACPI Patcher Error: Failed opening err2.txt \n"); err_exit(1);}
	errsize=ftell(f);
	fseek(f, 0, SEEK_SET);
	if (errsize != 0) {
		memset(errbuf, 0, sizeof(errbuf));
		fread(errbuf, 1, errsize, f);
		printf("ACPI Patcher Error: Failed patching %s from %s patch\n", p_tablename, filename2);
		printf("%s \n",errbuf);
		err_exit(1);
	}
	fclose(f);
	f = fopen("log2.txt", "r");
	if (!f) {printf("ACPI Patcher Error: Failed opening log2.txt \n"); err_exit(1);}
	if (fseek(f, 0, SEEK_END)) {printf("ACPI Patcher Error: Failed opening log2.txt \n"); err_exit(1);}
	errsize=ftell(f);
	fseek(f, 0, SEEK_SET);
	if (errsize != 0) {
		memset(errbuf, 0, sizeof(errbuf));
		fread(errbuf, 1, errsize, f);
		printf("ACPI Patcher Error: Failed patching %s from %s patch\n", p_tablename, filename2);
		printf("%s \n",errbuf);
		err_exit(1);
	}
	fclose(f);	
 
	char *compile_args[] = {
	"redir.exe",
	"-o",
	"log3.txt",
	"-e",
	"err3.txt",
	
	iasl,
	"-ve", // log errors only
	"target_p.dsl",
	0 };
	
	ret=spawnv(P_WAIT, "redir.exe", compile_args);
		i=0; while (compile_args[i] != NULL) { LOG("%s ", compile_args[i++]); }; LOG("\n");
		LOG("errorcode compile: %d \n", ret);
		
	f = fopen("err3.txt", "r");
	if (!f) {printf("ACPI Patcher Error: Failed opening err3.txt \n"); err_exit(1);}
	if (fseek(f, 0, SEEK_END)) {printf("ACPI Patcher Error: Failed opening err3.txt \n"); err_exit(1);}
	errsize=ftell(f);
	fseek(f, 0, SEEK_SET);
	if (errsize != 0) {
		memset(errbuf, 0, sizeof(errbuf));
		fread(errbuf, 1, errsize, f);
		printf("ACPI Patcher Error: Failed compiling %s after %s patch\n", p_tablename, filename2);
		printf("%s \n",errbuf);
		err_exit(1);
	}
	fclose(f);
	
	binpatch(p_tablename, "target_p.aml");

 }
 else {
	printf("ACPI Patcher Error: Table %s not found\n", p_tablename); err_exit(1);
 }

}
 
int is_patchhex (const char *name) {
 if (strstr(name,".hex") || strstr(name,".HEX")) { return 1;}
 return 0;
}

int is_patchbin (const char *name) {
 if (strstr(name,".bin") || strstr(name,".BIN")) { return 1;}
 return 0;
}

int is_patchdif (const char *name) {
 if (strstr(name,".dif") || strstr(name,".DIF")) { return 1;}
 return 0;
}

static int parser (void *user, const char *section, const char *value) {

 cur_file=(char *) value;
	printf("Acpi Patching [%6s] - %12s \n", section, cur_file);

 if (memcmp(section, "RSDP", 4) == 0 ||
     memcmp(section, "RSDT", 4) == 0 ||
     memcmp(section, "XSDT", 4) == 0 ||
     memcmp(section, "FACP", 4) == 0 ) {printf("ACPI Patcher Error: Patching %s is not implemented \n", section); err_exit(1); }
 if (strlen(section) > 6) {printf("ACPI Patcher Error: Section %s too long \n", section); err_exit(1); }

 if (is_patchbin(value)) {
	binpatch(section,value);
	return 1;
 }
 
 if (is_patchhex(value)) {
	hexpatch(section,value);
	return 1;
 }

 if (is_patchdif(value)) {
	difpatch(section,value);
	return 1;
 }

 printf("ACPI Patcher Error: filename %s mush have .hex, .bin or .dif extension \n", value); err_exit(1);
}

int main (int argc, char *argv[]) {
 int	error;

 shift_pressed=_bios_keybrd(_KEYBRD_SHIFTSTATUS) & 0x03; // only shift 
 file_log=fopen("main_log.txt","w");
 
 e820_init();
 acpi_init();

 ROOT_TREE=calloc(sizeof(tree_t),1);

 error = ini_parse("acpi_pat.cfg", parser, NULL);
 if (error < 0) {printf("ACPI Patcher Error: Cannot read 'acpi_pat.cfg' \n"); err_exit(1);}

 finish();
 
 shift_pressed |= (_bios_keybrd(_KEYBRD_SHIFTSTATUS) & 0x03); 
 if (shift_pressed)  err_exit(1); // debug exit
 
 make_grubmenu(); 
 fclose(file_log);
 
 return (0); // normal exit status=0
}
