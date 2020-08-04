#ifndef __QSTART_H
#define __QSTART_H

#define NORMALMODE 0
#define RECOVERYMODE 1
#define SYSTEM_MTD_NUMBER 19 //I.E /dev/mtd19 is systemfs
#define RECOVERYFS_MTD_NUMBER 15 //I.E /dev/mtd15 is recoveryfs

bool get_bootmode(unsigned int *mode);
#endif
