/* user2.bin @ 0x81000 */

/* Flash Map (512KB + 512KB), support 1MB/2MB/4MB SPI Flash */
/* |..|........................|.....|.....|..|........................|.....|....|                       */
/* ^  ^                        ^     ^     ^  ^                        ^     ^                            */
/* |_boot start(0x00000)       |     |     |_pad start(0x80000)        |     |                            */
/*    |_user1 start(0x01000)   |_user1 end    |_user2 start(0x81000)   |_user2 end                        */
/*                                   |_system param symmetric area(0x7c000)  |_system param area(0xfc000) */
/*                                                                            (last 4 sectors (16 kB))    */
/*    ------------ slot 1 -----------         ------------ slot 2 -----------                             */

/* NOTICE: */ 
/* 1. You can change irom0 len, but MUST make sure user2 end not overlap system param area. */
/* 2. Space between user2 end and system param area can be used as user param area.         */
/* 3. Don't change any other seg.                                                           */

MEMORY
{
  dport0_0_seg :                        org = 0x3FF00000, len = 0x10
  dram0_0_seg :                         org = 0x3FFE8000, len = 0x18000
  iram1_0_seg :                         org = 0x40100000, len = 0x8000
  irom0_0_seg :                         org = 0x40291000, len = 0x6B000 /* for 1 MB flash, FOTA slot 2 (second 512 kB). File 0x00000.bin will be flashed -> 0x81000 (max 64 kB), 0x91000.bin -> 0x91000 (max 428 kB) */
}

INCLUDE "../ld/eagle.app.v6.common.ld"
